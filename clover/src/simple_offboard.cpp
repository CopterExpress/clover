/*
 * Simplified copter control in OFFBOARD mode
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <algorithm>
#include <string>
#include <cmath>
#include <boost/format.hpp>
#include <stdexcept>
#include <GeographicLib/Geodesic.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/Altitude.h>

#include <clover/GetTelemetry.h>
#include <clover/Navigate.h>
#include <clover/NavigateGlobal.h>
#include <clover/SetAltitude.h>
#include <clover/SetYaw.h>
#include <clover/SetYawRate.h>
#include <clover/SetPosition.h>
#include <clover/SetVelocity.h>
#include <clover/SetAttitude.h>
#include <clover/SetRates.h>
#include <clover/State.h>

using std::string;
using std::isnan;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace clover;
using mavros_msgs::PositionTarget;
using mavros_msgs::AttitudeTarget;
using mavros_msgs::Thrust;
using mavros_msgs::Altitude;

// tf2
tf2_ros::Buffer tf_buffer;
std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;

// Parameters
string mavros;
string local_frame;
string fcu_frame;
ros::Duration transform_timeout;
ros::Duration telemetry_transform_timeout;
ros::Duration offboard_timeout;
ros::Duration land_timeout;
ros::Duration arming_timeout;
ros::Duration local_position_timeout;
ros::Duration state_timeout;
ros::Duration velocity_timeout;
ros::Duration global_position_timeout;
ros::Duration battery_timeout;
ros::Duration manual_control_timeout;
float default_speed;
bool auto_release;
bool land_only_in_offboard, nav_from_sp, check_kill_switch;
std::map<string, string> reference_frames;
string terrain_frame_mode;

// Publishers
ros::Publisher attitude_pub, attitude_raw_pub, position_pub, position_raw_pub, rates_pub, thrust_pub, state_pub;

// Service clients
ros::ServiceClient arming, set_mode;

// Containers
ros::Timer setpoint_timer;
PoseStamped position_msg;
PositionTarget position_raw_msg;
//TwistStamped rates_msg;
TransformStamped target, setpoint;
geometry_msgs::TransformStamped body;
geometry_msgs::TransformStamped terrain;

// State
PoseStamped nav_start;
PointStamped setpoint_position;
PointStamped setpoint_altitude;
Vector3Stamped setpoint_velocity;
float setpoint_yaw, setpoint_roll, setpoint_pitch;
Vector3 setpoint_rates;
string yaw_frame_id;
float setpoint_thrust;
float nav_speed;
float setpoint_lat = NAN, setpoint_lon = NAN;
bool busy = false;
bool wait_armed = false;
bool nav_from_sp_flag = false;

// Last published
PoseStamped setpoint_pose_local;
Vector3Stamped setpoint_velocity_local;
float yaw_local;

enum setpoint_type_t {
	NONE,
	NAVIGATE,
	NAVIGATE_GLOBAL,
	POSITION,
	VELOCITY,
	ATTITUDE,
	RATES,
	_ALTITUDE,
	_YAW,
	_YAW_RATE,
};

enum setpoint_type_t setpoint_type = NONE;

enum { YAW, YAW_RATE, TOWARDS } setpoint_yaw_type;

// Last received telemetry messages
mavros_msgs::State state;
mavros_msgs::StatusText statustext;
mavros_msgs::ManualControl manual_control;
PoseStamped local_position;
TwistStamped velocity;
NavSatFix global_position;
BatteryState battery;

// Common subscriber callback template that stores message to the variable
template<typename T, T& STORAGE>
void handleMessage(const T& msg)
{
	STORAGE = msg;
}

void handleState(const mavros_msgs::State& s)
{
	state = s;
	if (s.mode != "OFFBOARD") {
		// flight intercepted
		nav_from_sp_flag = false;
	}
}

inline void publishBodyFrame()
{
	if (body.child_frame_id.empty()) return;
	if (!body.header.stamp.isZero() && body.header.stamp == local_position.header.stamp) {
		return; // avoid TF_REPEATED_DATA warnings
	}

	tf::Quaternion q;
	q.setRPY(0, 0, tf::getYaw(local_position.pose.orientation));
	tf::quaternionTFToMsg(q, body.transform.rotation);

	body.transform.translation.x = local_position.pose.position.x;
	body.transform.translation.y = local_position.pose.position.y;
	body.transform.translation.z = local_position.pose.position.z;
	body.header.frame_id = local_position.header.frame_id;
	body.header.stamp = local_position.header.stamp;
	transform_broadcaster->sendTransform(body);
}

void handleLocalPosition(const PoseStamped& pose)
{
	local_position = pose;
	publishBodyFrame();
	// TODO: home?
}

// wait for transform without interrupting publishing setpoints
inline bool waitTransform(const string& target, const string& source,
                          const ros::Time& stamp, const ros::Duration& timeout) // editorconfig-checker-disable-line
{
	ros::Rate r(100);
	auto start = ros::Time::now();
	while (ros::ok()) {
		if (ros::Time::now() - start > timeout) return false;
		if (tf_buffer.canTransform(target, source, stamp)) return true;
		ros::spinOnce();
		r.sleep();
	}
	return false;
}

void publishTerrain(const double distance, const ros::Time& stamp)
{
	if (!waitTransform(local_frame, body.child_frame_id, stamp, ros::Duration(0.1))) return;

	auto t = tf_buffer.lookupTransform(local_frame, body.child_frame_id, stamp);
	t.child_frame_id = terrain.child_frame_id;
	t.transform.translation.z -= distance;
	static_transform_broadcaster->sendTransform(t);
}

void handleAltitude(const Altitude& alt)
{
	if (!std::isfinite(alt.bottom_clearance)) return;
	publishTerrain(alt.bottom_clearance, alt.header.stamp);
}

void handleRange(const Range& range)
{
	if (!std::isfinite(range.range)) return;
	// TODO: check it's facing down
	publishTerrain(range.range, range.header.stamp);
}

#define TIMEOUT(msg, timeout) (msg.header.stamp.isZero() || (ros::Time::now() - msg.header.stamp > timeout))

bool getTelemetry(GetTelemetry::Request& req, GetTelemetry::Response& res)
{
	ros::Time stamp = ros::Time::now();

	if (req.frame_id.empty())
		req.frame_id = local_frame;

	res.frame_id = req.frame_id;
	res.x = NAN;
	res.y = NAN;
	res.z = NAN;
	res.lat = NAN;
	res.lon = NAN;
	res.alt = NAN;
	res.vx = NAN;
	res.vy = NAN;
	res.vz = NAN;
	res.roll = NAN;
	res.pitch = NAN;
	res.yaw = NAN;
	res.roll_rate = NAN;
	res.pitch_rate = NAN;
	res.yaw_rate = NAN;
	res.voltage = NAN;
	res.cell_voltage = NAN;

	if (!TIMEOUT(state, state_timeout)) {
		res.connected = state.connected;
		res.armed = state.armed;
		res.mode = state.mode;
	}

	try {
		waitTransform(req.frame_id, fcu_frame, stamp, telemetry_transform_timeout);
		auto transform = tf_buffer.lookupTransform(req.frame_id, fcu_frame, stamp);
		res.x = transform.transform.translation.x;
		res.y = transform.transform.translation.y;
		res.z = transform.transform.translation.z;

		double yaw, pitch, roll;
		tf2::getEulerYPR(transform.transform.rotation, yaw, pitch, roll);
		res.yaw = yaw;
		res.pitch = pitch;
		res.roll = roll;
	} catch (const tf2::TransformException& e) {
		ROS_DEBUG("%s", e.what());
	}

	if (!TIMEOUT(velocity, velocity_timeout)) {
		try {
			// transform velocity
			waitTransform(req.frame_id, fcu_frame, velocity.header.stamp, telemetry_transform_timeout);
			Vector3Stamped vec, vec_out;
			vec.header.stamp = velocity.header.stamp;
			vec.header.frame_id = velocity.header.frame_id;
			vec.vector = velocity.twist.linear;
			tf_buffer.transform(vec, vec_out, req.frame_id);

			res.vx = vec_out.vector.x;
			res.vy = vec_out.vector.y;
			res.vz = vec_out.vector.z;
		} catch (const tf2::TransformException& e) {}

		// use angular velocities as they are
		res.yaw_rate = velocity.twist.angular.z;
		res.pitch_rate = velocity.twist.angular.y;
		res.roll_rate = velocity.twist.angular.x;
	}

	if (!TIMEOUT(global_position, global_position_timeout)) {
		res.lat = global_position.latitude;
		res.lon = global_position.longitude;
		res.alt = global_position.altitude;
	}

	if (!TIMEOUT(battery, battery_timeout)) {
		res.voltage = battery.voltage;
		if (!battery.cell_voltage.empty()) {
			res.cell_voltage = battery.cell_voltage[0];
		}
	}

	return true;
}

// throws std::runtime_error
void offboardAndArm()
{
	ros::Rate r(10);

	if (state.mode != "OFFBOARD") {
		auto start = ros::Time::now();
		ROS_INFO("switch to OFFBOARD");
		static mavros_msgs::SetMode sm;
		sm.request.custom_mode = "OFFBOARD";

		if (!set_mode.call(sm))
			throw std::runtime_error("Error calling set_mode service");

		// wait for OFFBOARD mode
		while (ros::ok()) {
			ros::spinOnce();
			if (state.mode == "OFFBOARD") {
				break;
			} else if (ros::Time::now() - start > offboard_timeout) {
				string report = "OFFBOARD timed out";
				if (statustext.header.stamp > start)
					report += ": " + statustext.text;
				throw std::runtime_error(report);
			}
			ros::spinOnce();
			r.sleep();
		}
	}

	if (!state.armed) {
		ros::Time start = ros::Time::now();
		ROS_INFO("arming");
		mavros_msgs::CommandBool srv;
		srv.request.value = true;
		if (!arming.call(srv)) {
			throw std::runtime_error("Error calling arming service");
		}

		// wait until armed
		while (ros::ok()) {
			ros::spinOnce();
			if (state.armed) {
				break;
			} else if (ros::Time::now() - start > arming_timeout) {
				string report = "Arming timed out";
				if (statustext.header.stamp > start)
					report += ": " + statustext.text;
				throw std::runtime_error(report);
			}
			ros::spinOnce();
			r.sleep();
		}
	}
}

inline double hypot(double x, double y, double z)
{
	return std::sqrt(x * x + y * y + z * z);
}

inline float getDistance(const Point& from, const Point& to)
{
	return hypot(from.x - to.x, from.y - to.y, from.z - to.z);
}

void getNavigateSetpoint(const ros::Time& stamp, const float speed, Point& nav_setpoint)
{
	if (wait_armed) {
		// don't start navigating if we're waiting arming
		nav_start.header.stamp = stamp;
	}

	float distance = getDistance(nav_start.pose.position, setpoint_pose_local.pose.position);
	float time = distance / speed;
	float passed = std::min((stamp - nav_start.header.stamp).toSec() / time, 1.0);

	nav_setpoint.x = nav_start.pose.position.x + (setpoint_pose_local.pose.position.x - nav_start.pose.position.x) * passed;
	nav_setpoint.y = nav_start.pose.position.y + (setpoint_pose_local.pose.position.y - nav_start.pose.position.y) * passed;
	nav_setpoint.z = nav_start.pose.position.z + (setpoint_pose_local.pose.position.z - nav_start.pose.position.z) * passed;
}

PoseStamped globalToLocal(double lat, double lon)
{
	auto earth = GeographicLib::Geodesic::WGS84();

	// Determine azimuth and distance between current and destination point
	double _, distance, azimuth;
	earth.Inverse(global_position.latitude, global_position.longitude, lat, lon, distance, _, azimuth);

	double x_offset, y_offset;
	double azimuth_radians = azimuth * M_PI / 180;
	x_offset = distance * sin(azimuth_radians);
	y_offset = distance * cos(azimuth_radians);

	if (!waitTransform(local_frame, fcu_frame, global_position.header.stamp, ros::Duration(0.2))) {
		throw std::runtime_error("No local position");
	}

	auto local = tf_buffer.lookupTransform(local_frame, fcu_frame, global_position.header.stamp);

	PoseStamped pose;
	pose.header.stamp = global_position.header.stamp; // TODO: ?
	pose.header.frame_id = local_frame;
	pose.pose.position.x = local.transform.translation.x + x_offset;
	pose.pose.position.y = local.transform.translation.y + y_offset;
	pose.pose.orientation.w = 1;
	return pose;
}

// publish navigate_target frame
void publishTarget(ros::Time stamp, bool _static = false)
{
	bool single_frame = (setpoint_position.header.frame_id == setpoint_altitude.header.frame_id);

	// handle yaw for target frame
	if (setpoint_yaw_type == YAW || setpoint_yaw_type == YAW_RATE) { // use last set yaw for yaw_rate
		if (setpoint_altitude.header.frame_id == yaw_frame_id) {
			target.transform.rotation = tf::createQuaternionMsgFromYaw(setpoint_yaw);
		} else {
			single_frame = false;
			target.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_local);
		}
	} else if (setpoint_yaw_type == TOWARDS) {
		single_frame = false;
		target.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_local);
	}

	if (_static && single_frame) {
		// publish at user's command, if all frames are the same
		target.header.frame_id = setpoint_position.header.frame_id;
		target.header.stamp = stamp;
		target.transform.translation.x = setpoint_position.point.x;
		target.transform.translation.y = setpoint_position.point.y;
		target.transform.translation.z = setpoint_position.point.z;

	} else if (!_static) {
		// publish at each iteration, if frames are different
		target.header = setpoint_pose_local.header;
		target.transform.translation.x = setpoint_pose_local.pose.position.x;
		target.transform.translation.y = setpoint_pose_local.pose.position.y;
		target.transform.translation.z = setpoint_pose_local.pose.position.z;
	}

	static_transform_broadcaster->sendTransform(target);
}

void publish(const ros::Time stamp)
{
	if (setpoint_type == NONE) return;

	position_raw_msg.header.stamp = stamp;

	// transform position
	if (setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL || setpoint_type == POSITION) {
		setpoint_position.header.stamp = stamp;
		setpoint_altitude.header.stamp = stamp;
		// transform xy
		try {
			auto xy = tf_buffer.transform(setpoint_position, local_frame, ros::Duration(0.05));
			setpoint_pose_local.header = xy.header;
			setpoint_pose_local.pose.position.x = xy.point.x;
			setpoint_pose_local.pose.position.y = xy.point.y;
		} catch (tf2::TransformException& ex) {
			// can't transform xy, use last known
			ROS_WARN_THROTTLE(10, "can't transform: %s", ex.what());
		}
		// transform altitude
		try {
			setpoint_pose_local.pose.position.z = tf_buffer.transform(setpoint_altitude, local_frame, ros::Duration(0.05)).point.z;
		} catch (tf2::TransformException& ex) {
			// can't transform altitude, use last known
			ROS_WARN_THROTTLE(10, "can't transform: %s", ex.what());
		}
	}

	// transform yaw
	if (setpoint_yaw_type == YAW) {
		try {
			QuaternionStamped q;
			q.header.stamp = stamp;
			q.header.frame_id = yaw_frame_id;
			q.quaternion = tf::createQuaternionMsgFromYaw(setpoint_yaw);
			yaw_local = tf2::getYaw(tf_buffer.transform(q, local_frame, ros::Duration(0.05)).quaternion);
		} catch (tf2::TransformException& ex) {
			// can't transform yaw, use last known
			ROS_WARN_THROTTLE(10, "can't transform: %s", ex.what());
		}
	}

	// compute navigate setpoint
	if (setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL) {
		getNavigateSetpoint(stamp, nav_speed, position_msg.pose.position);

		if (setpoint_yaw_type == TOWARDS) {
			yaw_local = atan2(position_msg.pose.position.y - nav_start.pose.position.y,
			                  position_msg.pose.position.x - nav_start.pose.position.x);
		}

		position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_local);
	}

	if (setpoint_type == POSITION) {
		position_msg = setpoint_pose_local;
		position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_local);
	}

	if (setpoint_type == POSITION || setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL) {
		position_msg.header.stamp = stamp;

		if (setpoint_yaw_type == YAW || setpoint_yaw_type == TOWARDS) {
			position_pub.publish(position_msg);

		} else {
			position_raw_msg.type_mask = PositionTarget::IGNORE_VX +
			                             PositionTarget::IGNORE_VY +
			                             PositionTarget::IGNORE_VZ +
			                             PositionTarget::IGNORE_AFX +
			                             PositionTarget::IGNORE_AFY +
			                             PositionTarget::IGNORE_AFZ +
			                             PositionTarget::IGNORE_YAW;
			position_raw_msg.yaw_rate = setpoint_rates.z;
			position_raw_msg.position = position_msg.pose.position;
			position_raw_pub.publish(position_raw_msg);
		}

		// publish setpoint frame
		if (!setpoint.child_frame_id.empty()) {
			if (setpoint.header.stamp >= position_msg.header.stamp) {
				return; // avoid TF_REPEATED_DATA warnings
			}

			setpoint.transform.translation.x = position_msg.pose.position.x;
			setpoint.transform.translation.y = position_msg.pose.position.y;
			setpoint.transform.translation.z = position_msg.pose.position.z;
			setpoint.transform.rotation = position_msg.pose.orientation;
			setpoint.header.frame_id = position_msg.header.frame_id;
			setpoint.header.stamp = position_msg.header.stamp;
			transform_broadcaster->sendTransform(setpoint);
		}

		// publish dynamic target frame
		publishTarget(stamp);
	}

	if (setpoint_type == VELOCITY) {
		// transform velocity to local frame
		setpoint_velocity.header.stamp = stamp;
		try {
			setpoint_velocity_local = tf_buffer.transform(setpoint_velocity, local_frame, ros::Duration(0.05));
		} catch (tf2::TransformException& ex) {
			// can't transform velocity, use last known
			ROS_WARN_THROTTLE(10, "can't transform: %s", ex.what());
		}

		// publish velocity
		position_raw_msg.type_mask = PositionTarget::IGNORE_PX +
		                             PositionTarget::IGNORE_PY +
		                             PositionTarget::IGNORE_PZ +
		                             PositionTarget::IGNORE_AFX +
		                             PositionTarget::IGNORE_AFY +
		                             PositionTarget::IGNORE_AFZ;
		position_raw_msg.type_mask += setpoint_yaw_type == YAW ? PositionTarget::IGNORE_YAW_RATE : PositionTarget::IGNORE_YAW;
		position_raw_msg.velocity = setpoint_velocity_local.vector;
		position_raw_msg.yaw = yaw_local;
		position_raw_msg.yaw_rate = setpoint_rates.z;
		position_raw_pub.publish(position_raw_msg);
	}

	if (setpoint_type == ATTITUDE) {
		PoseStamped msg;
		msg.header.stamp = stamp;
		msg.header.frame_id = local_frame;
		msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(setpoint_roll, setpoint_pitch, yaw_local);
		attitude_pub.publish(msg);

		Thrust thrust_msg;
		thrust_msg.header.stamp = stamp;
		thrust_msg.thrust = setpoint_thrust;
		thrust_pub.publish(thrust_msg);
	}

	if (setpoint_type == RATES) {
		// rates_pub.publish(rates_msg);
		// thrust_pub.publish(thrust_msg);
		// mavros rates topics waits for rates in local frame
		// use rates in body frame for simplicity
		AttitudeTarget att_raw_msg;
		att_raw_msg.header.stamp = stamp;
		att_raw_msg.header.frame_id = fcu_frame;
		att_raw_msg.type_mask = AttitudeTarget::IGNORE_ATTITUDE;
		att_raw_msg.body_rate = setpoint_rates;
		att_raw_msg.thrust = setpoint_thrust;
		attitude_raw_pub.publish(att_raw_msg);
	}
}

void publishSetpoint(const ros::TimerEvent& event)
{
	publish(event.current_real);
}

inline void checkManualControl()
{
	if (!manual_control_timeout.isZero() && TIMEOUT(manual_control, manual_control_timeout)) {
		throw std::runtime_error("Manual control timeout, RC is switched off?");
	}

	if (check_kill_switch) {
		// switch values: https://github.com/PX4/PX4-Autopilot/blob/c302514a0809b1765fafd13c014d705446ae1113/msg/manual_control_setpoint.msg#L3
		const uint8_t SWITCH_POS_NONE = 0; // switch is not mapped
		const uint8_t SWITCH_POS_ON = 1; // switch activated
		const uint8_t SWITCH_POS_MIDDLE = 2; // middle position
		const uint8_t SWITCH_POS_OFF = 3; // switch not activated

		const int KILL_SWITCH_BIT = 12; // https://github.com/PX4/Firmware/blob/c302514a0809b1765fafd13c014d705446ae1113/src/modules/mavlink/mavlink_messages.cpp#L3975
		uint8_t kill_switch = (manual_control.buttons & (0b11 << KILL_SWITCH_BIT)) >> KILL_SWITCH_BIT;

		if (kill_switch == SWITCH_POS_ON)
			throw std::runtime_error("Kill switch is on");
	}
}

inline void checkState()
{
	if (TIMEOUT(state, state_timeout))
		throw std::runtime_error("State timeout, check mavros settings");

	if (!state.connected)
		throw std::runtime_error("No connection to FCU, https://clover.coex.tech/connection");
}

void publishState()
{
	clover::State msg;
	msg.mode = setpoint_type;
	msg.yaw_mode = setpoint_yaw_type;

	if (setpoint_position.header.frame_id.empty()) {
		msg.x = NAN;
		msg.y = NAN;
		msg.z = NAN;
	} else {
		msg.x = setpoint_position.point.x;
		msg.y = setpoint_position.point.y;
		msg.z = setpoint_altitude.point.z;
	}

	msg.speed = nav_speed;
	msg.lat = setpoint_lat;
	msg.lon = setpoint_lon;
	msg.vx = setpoint_velocity.vector.x;
	msg.vy = setpoint_velocity.vector.y;
	msg.vz = setpoint_velocity.vector.z;
	msg.roll = setpoint_roll;
	msg.pitch = setpoint_pitch;
	msg.yaw = !yaw_frame_id.empty() ? setpoint_yaw : NAN;

	msg.roll_rate = setpoint_rates.x;
	msg.pitch_rate = setpoint_rates.y;
	msg.yaw_rate = setpoint_rates.z;
	msg.thrust = setpoint_thrust;

	if (setpoint_type == VELOCITY) {
		msg.xy_frame_id = setpoint_velocity.header.frame_id;
		msg.z_frame_id = setpoint_velocity.header.frame_id;
	} else {
		msg.xy_frame_id = setpoint_position.header.frame_id;
		msg.z_frame_id = setpoint_altitude.header.frame_id;
	}
	msg.yaw_frame_id = yaw_frame_id;

	state_pub.publish(msg);
}

inline float safe(float value) {
	return std::isfinite(value) ? value : 0;
}

#define ENSURE_FINITE(var) { if (!std::isfinite(var)) throw std::runtime_error(#var " argument cannot be NaN or Inf"); }

#define ENSURE_NON_INF(var) { if (std::isinf(var)) throw std::runtime_error(#var " argument cannot be Inf"); }

bool serve(enum setpoint_type_t sp_type, float x, float y, float z, float vx, float vy, float vz,
           float roll, float pitch, float yaw, float roll_rate, float pitch_rate, float yaw_rate, // editorconfig-checker-disable-line
           float lat, float lon, float thrust, float speed, string frame_id, bool auto_arm, // editorconfig-checker-disable-line
           uint8_t& success, string& message) // editorconfig-checker-disable-line
{
	auto stamp = ros::Time::now();

	try {
		if (busy)
			throw std::runtime_error("Busy");

		busy = true;

		// Checks
		checkState();

		if (auto_arm) {
			checkManualControl();
		}

		// default frame is local frame
		if (frame_id.empty())
			frame_id = local_frame;

		// look up for reference frame
		auto search = reference_frames.find(frame_id);
		const string& reference_frame = search == reference_frames.end() ? frame_id : search->second;

		ENSURE_NON_INF(x);
		ENSURE_NON_INF(y);
		ENSURE_NON_INF(z);
		ENSURE_NON_INF(speed); // TODO: allow inf
		ENSURE_NON_INF(vx);
		ENSURE_NON_INF(vy);
		ENSURE_NON_INF(vz);
		ENSURE_NON_INF(roll);
		ENSURE_NON_INF(pitch);
		ENSURE_NON_INF(roll_rate);
		ENSURE_NON_INF(pitch_rate);
		ENSURE_NON_INF(yaw_rate);
		ENSURE_NON_INF(thrust);

		if (sp_type == NAVIGATE_GLOBAL) {
			ENSURE_FINITE(lat);
			ENSURE_FINITE(lon);
		}

		if (isfinite(x) != isfinite(y)) {
			throw std::runtime_error("x and y can be set only together");
		}

		if (isfinite(yaw_rate)) {
			if (sp_type > RATES && setpoint_type == ATTITUDE) {
				throw std::runtime_error("Yaw rate cannot be set in attitude mode.");
			}
		}

		// set_altitude
		if (sp_type == _ALTITUDE) {
			if (setpoint_type == VELOCITY || setpoint_type == ATTITUDE || setpoint_type == RATES) {
				throw std::runtime_error("Altitude cannot be set in velocity, attitude or rates mode.");
			}
		}

		if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL) {
			if (TIMEOUT(local_position, local_position_timeout))
				throw std::runtime_error("No local position, check settings");

			if (speed < 0)
				throw std::runtime_error("Navigate speed must be positive, " + std::to_string(speed) + " passed");

			if (speed == 0)
				speed = default_speed;
		}

		if (sp_type == NAVIGATE_GLOBAL) {
			if (TIMEOUT(global_position, global_position_timeout))
				throw std::runtime_error("No global position");
		}

		// if any value need to be transformed to reference frame
		if (isfinite(x) || isfinite(y) || isfinite(z) || isfinite(vx) || isfinite(vy) || isfinite(vz) || isfinite(yaw)) {
			// make sure transform from frame_id to reference frame available
			if (!waitTransform(reference_frame, frame_id, stamp, transform_timeout))
				throw std::runtime_error("Can't transform from " + frame_id + " to " + reference_frame);

			// make sure transform from reference frame to local frame available
			if (!waitTransform(local_frame, reference_frame, stamp, transform_timeout))
				throw std::runtime_error("Can't transform from " + reference_frame + " to " + local_frame);
		}

		if (sp_type == NAVIGATE_GLOBAL) {
			// Calculate x and from lat and lot in request's frame
			auto pose_local = globalToLocal(lat, lon);
			pose_local.header.stamp = stamp; // TODO: fix
			auto xy_in_req_frame = tf_buffer.transform(pose_local, frame_id);
			x = xy_in_req_frame.pose.position.x;
			y = xy_in_req_frame.pose.position.y;
			setpoint_lat = lat;
			setpoint_lon = lon;
		}

		// Everything fine - switch setpoint type
		if (sp_type <= RATES) {
			setpoint_type = sp_type;
		}

		if (setpoint_type != NAVIGATE && setpoint_type != NAVIGATE_GLOBAL) {
			nav_from_sp_flag = false;
		}

		bool to_auto_arm = auto_arm && (state.mode != "OFFBOARD" || !state.armed);
		if (to_auto_arm || setpoint_type == VELOCITY || setpoint_type == ATTITUDE || setpoint_type == RATES) {
			// invalidate position setpoint
			setpoint_position.header.frame_id = "";
			setpoint_altitude.header.frame_id = "";
			yaw_frame_id = "";
		}

		if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL) {
			// starting point
			if (nav_from_sp && nav_from_sp_flag) {
				message = "Navigating from current setpoint";
				nav_start = position_msg;
			} else {
				nav_start = local_position;
			}

			if (!isnan(speed)) {
				nav_speed = speed;
			}

			nav_from_sp_flag = true;
		}

		// handle position
		if (setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL || setpoint_type == POSITION) {

			PointStamped desired;
			desired.header.frame_id = frame_id;
			desired.header.stamp = stamp;
			desired.point.x = safe(x);
			desired.point.y = safe(y);
			desired.point.z = safe(z);

			// transform to reference frame
			desired = tf_buffer.transform(desired, reference_frame);

			// set horizontal position
			if (isfinite(x) && isfinite(y)) {
				setpoint_position = desired;
			} else if (setpoint_position.header.frame_id.empty()) {
				 // TODO: use transform for current stamp
				setpoint_position.header = local_position.header;
				setpoint_position.point = local_position.pose.position;
			}

			// set altitude
			if (isfinite(z)) {
				setpoint_altitude = desired;
			} else if (setpoint_altitude.header.frame_id.empty()) {
				setpoint_altitude.header = local_position.header;
				setpoint_altitude.point = local_position.pose.position;
			}
		}

		// handle velocity
		if (sp_type == VELOCITY) {
			// TODO: allow setting different modes by altitude and xy
			Vector3Stamped desired;
			desired.header.frame_id = frame_id;
			desired.header.stamp = stamp;
			desired.vector.x = safe(vx);
			desired.vector.y = safe(vy);
			desired.vector.z = safe(vz);

			// transform to reference frame
			desired = tf_buffer.transform(desired, reference_frame);
			setpoint_velocity.header = desired.header;

			// set horizontal velocity
			if (isfinite(vx) && isfinite(vy)) {
				setpoint_velocity.vector.x = desired.vector.x;
				setpoint_velocity.vector.y = desired.vector.y;
			}

			// set vertical velocity
			if (isfinite(vz)) {
				setpoint_velocity.vector.z = desired.vector.z;
			}
		}

		// handle yaw
		if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL || sp_type == POSITION || sp_type == VELOCITY || sp_type == ATTITUDE || sp_type == _YAW) {
			if (isfinite(yaw)) {
				setpoint_yaw_type = YAW;
				QuaternionStamped desired;
				desired.header.frame_id = frame_id;
				desired.header.stamp = stamp;
				desired.quaternion = tf::createQuaternionMsgFromYaw(yaw);

				// transform to reference frame
				desired = tf_buffer.transform(desired, reference_frame);
				setpoint_yaw = tf2::getYaw(desired.quaternion);
				yaw_frame_id = reference_frame;

			} else if (isinf(yaw) && yaw > 0) {
				// yaw towards
				setpoint_yaw_type = TOWARDS;

			} else if (yaw_frame_id.empty() || sp_type == _YAW) {
				// yaw is nan and not set previously OR set_yaw(yaw=nan) was called
				setpoint_yaw_type = YAW;
				setpoint_yaw = tf2::getYaw(local_position.pose.orientation); // set yaw to current yaw
				yaw_frame_id = local_position.header.frame_id;
			}
		}

		// handle roll
		if (isfinite(roll)) {
			setpoint_roll = roll;
		}

		// handle pitch
		if (isfinite(pitch)) {
			setpoint_pitch = pitch;
		}

		// handle yaw rate
		if (isfinite(yaw_rate)) {
			setpoint_yaw_type = YAW_RATE;
			setpoint_rates.z = yaw_rate;
		}

		// handle pitch rate
		if (isfinite(roll_rate)) {
			setpoint_rates.x = roll_rate;
		}

		// handle roll rate
		if (isfinite(pitch_rate)) {
			setpoint_rates.y = pitch_rate;
		}

		// handle thrust
		if (isfinite(thrust)) {
			setpoint_thrust = thrust;
		}

		wait_armed = auto_arm;

		publish(stamp); // calculate initial transformed messages first
		setpoint_timer.start();

		if (setpoint_type == NAVIGATE || setpoint_type == NAVIGATE_GLOBAL || setpoint_type == POSITION) {
			publishTarget(stamp, true);
		}

		publishState();

		if (auto_arm) {
			offboardAndArm();
			wait_armed = false;
		} else if (state.mode != "OFFBOARD") {
			setpoint_timer.stop();
			throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
		} else if (!state.armed) {
			setpoint_timer.stop();
			throw std::runtime_error("Copter is not armed, use auto_arm?");
		}

	} catch (const std::exception& e) {
		message = e.what();
		ROS_INFO("%s", message.c_str());
		busy = false;
		return true;
	}

	success = true;
	busy = false;
	return true;
}

bool navigate(Navigate::Request& req, Navigate::Response& res) {
	return serve(NAVIGATE, req.x, req.y, req.z, NAN, NAN, NAN, NAN, NAN, req.yaw, NAN, NAN, NAN, NAN, NAN, NAN, req.speed, req.frame_id, req.auto_arm, res.success, res.message);
}

bool navigateGlobal(NavigateGlobal::Request& req, NavigateGlobal::Response& res) {
	return serve(NAVIGATE_GLOBAL, NAN, NAN, req.z, NAN, NAN, NAN, NAN, NAN, req.yaw, NAN, NAN, NAN, req.lat, req.lon, NAN, req.speed, req.frame_id, req.auto_arm, res.success, res.message);
}

bool setAltitude(SetAltitude::Request& req, SetAltitude::Response& res) {
	return serve(_ALTITUDE, NAN, NAN, req.z, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req.frame_id, false, res.success, res.message);
}

bool setYaw(SetYaw::Request& req, SetYaw::Response& res) {
	return serve(_YAW, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req.yaw, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req.frame_id, false, res.success, res.message);
}

bool setYawRate(SetYawRate::Request& req, SetYawRate::Response& res) {
	return serve(_YAW_RATE, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req.yaw_rate, NAN, NAN, NAN, NAN, "", false, res.success, res.message);
}

bool setPosition(SetPosition::Request& req, SetPosition::Response& res) {
	return serve(POSITION, req.x, req.y, req.z, NAN, NAN, NAN, NAN, NAN, req.yaw, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req.frame_id, req.auto_arm, res.success, res.message);
}

bool setVelocity(SetVelocity::Request& req, SetVelocity::Response& res) {
	return serve(VELOCITY, NAN, NAN, NAN, req.vx, req.vy, req.vz, NAN, NAN, req.yaw, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req.frame_id, req.auto_arm, res.success, res.message);
}

bool setAttitude(SetAttitude::Request& req, SetAttitude::Response& res) {
	return serve(ATTITUDE, NAN, NAN, NAN, NAN, NAN, NAN, req.roll, req.pitch, req.yaw, NAN, NAN, NAN, NAN, NAN, req.thrust, NAN, req.frame_id, req.auto_arm, res.success, res.message);
}

bool setRates(SetRates::Request& req, SetRates::Response& res) {
	return serve(RATES, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req.roll_rate, req.pitch_rate, req.yaw_rate, NAN, NAN, req.thrust, NAN, "", req.auto_arm, res.success, res.message);
}

bool land(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
	try {
		if (busy)
			throw std::runtime_error("Busy");

		busy = true;

		checkState();

		if (land_only_in_offboard) {
			if (state.mode != "OFFBOARD") {
				throw std::runtime_error("Copter is not in OFFBOARD mode");
			}
		}

		static mavros_msgs::SetMode sm;
		sm.request.custom_mode = "AUTO.LAND";

		if (!set_mode.call(sm))
			throw std::runtime_error("Can't call set_mode service");

		if (!sm.response.mode_sent)
			throw std::runtime_error("Can't send set_mode request");

		static ros::Rate r(10);
		auto start = ros::Time::now();
		while (ros::ok()) {
			if (state.mode == "AUTO.LAND") {
				break;
			}
			if (ros::Time::now() - start > land_timeout)
				throw std::runtime_error("Land request timed out");

			ros::spinOnce();
			r.sleep();
		}

		// stop setpoints and invalidate position setpoint
		setpoint_timer.stop();
		setpoint_type = NONE;
		setpoint_position.header.frame_id = "";
		setpoint_altitude.header.frame_id = "";
		yaw_frame_id = "";
		publishState();

		res.success = true;
		busy = false;
		return true;

	} catch (const std::exception& e) {
		res.message = e.what();
		ROS_INFO("%s", e.what());
		busy = false;
		return true;
	}
	return false;
}

bool release(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
	setpoint_timer.stop();
	setpoint_type = NONE;
	setpoint_position.header.frame_id = "";
	setpoint_altitude.header.frame_id = "";
	yaw_frame_id = "";
	publishState();
	res.success = true;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simple_offboard");
	ros::NodeHandle nh, nh_priv("~");

	tf2_ros::TransformListener tf_listener(tf_buffer);
	transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();
	static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

	// Params
	nh_priv.param("mavros", mavros, string("mavros")); // for case of using multiple connections
	nh.param<string>(mavros + "/local_position/tf/frame_id", local_frame, "map");
	nh.param<string>(mavros + "/local_position/tf/child_frame_id", fcu_frame, "base_link");
	nh_priv.param("target_frame", target.child_frame_id, string("navigate_target"));
	nh_priv.param("setpoint", setpoint.child_frame_id, string("setpoint"));
	nh_priv.param("auto_release", auto_release, true);
	nh_priv.param("land_only_in_offboard", land_only_in_offboard, true);
	nh_priv.param("nav_from_sp", nav_from_sp, true);
	nh_priv.param("check_kill_switch", check_kill_switch, true);
	nh_priv.param("default_speed", default_speed, 0.5f);
	nh_priv.param<string>("body_frame", body.child_frame_id, "body");
	nh_priv.param<string>("terrain_frame", terrain.child_frame_id, "terrain");
	nh_priv.param<string>("terrain_frame_mode", terrain_frame_mode, "altitude");
	nh_priv.getParam("reference_frames", reference_frames);

	// Default reference frames
	std::map<string, string> default_reference_frames;
	default_reference_frames[body.child_frame_id] = local_frame;
	default_reference_frames[fcu_frame] = local_frame;
	if (!target.child_frame_id.empty()) default_reference_frames[target.child_frame_id] = local_frame;
	reference_frames.insert(default_reference_frames.begin(), default_reference_frames.end()); // merge defaults

	state_timeout = ros::Duration(nh_priv.param("state_timeout", 3.0));
	local_position_timeout = ros::Duration(nh_priv.param("local_position_timeout", 2.0));
	velocity_timeout = ros::Duration(nh_priv.param("velocity_timeout", 2.0));
	global_position_timeout = ros::Duration(nh_priv.param("global_position_timeout", 10.0));
	battery_timeout = ros::Duration(nh_priv.param("battery_timeout", 2.0));
	manual_control_timeout = ros::Duration(nh_priv.param("manual_control_timeout", 0.0));

	transform_timeout = ros::Duration(nh_priv.param("transform_timeout", 0.5));
	telemetry_transform_timeout = ros::Duration(nh_priv.param("telemetry_transform_timeout", 0.5));
	offboard_timeout = ros::Duration(nh_priv.param("offboard_timeout", 3.0));
	land_timeout = ros::Duration(nh_priv.param("land_timeout", 3.0));
	arming_timeout = ros::Duration(nh_priv.param("arming_timeout", 4.0));

	// Service clients
	arming = nh.serviceClient<mavros_msgs::CommandBool>(mavros + "/cmd/arming");
	set_mode = nh.serviceClient<mavros_msgs::SetMode>(mavros + "/set_mode");

	// Telemetry subscribers
	auto state_sub = nh.subscribe(mavros + "/state", 1, &handleState);
	auto velocity_sub = nh.subscribe(mavros + "/local_position/velocity_body", 1, &handleMessage<TwistStamped, velocity>);
	auto global_position_sub = nh.subscribe(mavros + "/global_position/global", 1, &handleMessage<NavSatFix, global_position>);
	auto battery_sub = nh.subscribe(mavros + "/battery", 1, &handleMessage<BatteryState, battery>);
	auto statustext_sub = nh.subscribe(mavros + "/statustext/recv", 1, &handleMessage<mavros_msgs::StatusText, statustext>);
	auto manual_control_sub = nh.subscribe(mavros + "/manual_control/control", 1, &handleMessage<mavros_msgs::ManualControl, manual_control>);
	auto local_position_sub = nh.subscribe(mavros + "/local_position/pose", 1, &handleLocalPosition);

	ros::Subscriber altitude_sub;
	if (!body.child_frame_id.empty() && !terrain.child_frame_id.empty()) {
		terrain.header.frame_id = local_frame;
		if (terrain_frame_mode == "altitude") {
			altitude_sub = nh.subscribe(mavros + "/altitude", 1, &handleAltitude);
		} else if (terrain_frame_mode == "range") {
			string range_topic = nh_priv.param("range_topic", string("rangefinder/range"));
			altitude_sub = nh.subscribe(range_topic, 1, &handleRange);
		} else {
			ROS_FATAL("Unknown terrain_frame_mode: %s, valid values: altitude, range", terrain_frame_mode.c_str());
			ros::shutdown();
		}
	}

	// Setpoint publishers
	position_pub = nh.advertise<PoseStamped>(mavros + "/setpoint_position/local", 1);
	position_raw_pub = nh.advertise<PositionTarget>(mavros + "/setpoint_raw/local", 1);
	attitude_pub = nh.advertise<PoseStamped>(mavros + "/setpoint_attitude/attitude", 1);
	attitude_raw_pub = nh.advertise<AttitudeTarget>(mavros + "/setpoint_raw/attitude", 1);
	rates_pub = nh.advertise<TwistStamped>(mavros + "/setpoint_attitude/cmd_vel", 1);
	thrust_pub = nh.advertise<Thrust>(mavros + "/setpoint_attitude/thrust", 1);

	// State publisher
	state_pub = nh_priv.advertise<clover::State>("state", 1, true);

	 // Service servers
	auto gt_serv = nh.advertiseService("get_telemetry", &getTelemetry);
	auto na_serv = nh.advertiseService("navigate", &navigate);
	auto ng_serv = nh.advertiseService("navigate_global", &navigateGlobal);
	auto sl_serv = nh.advertiseService("set_altitude", &setAltitude);
	auto ya_serv = nh.advertiseService("set_yaw", &setYaw);
	auto yr_serv = nh.advertiseService("set_yaw_rate", &setYawRate);
	auto sp_serv = nh.advertiseService("set_position", &setPosition);
	auto sv_serv = nh.advertiseService("set_velocity", &setVelocity);
	auto sa_serv = nh.advertiseService("set_attitude", &setAttitude);
	auto sr_serv = nh.advertiseService("set_rates", &setRates);
	auto ld_serv = nh.advertiseService("land", &land);
	auto rl_serv = nh_priv.advertiseService("release", &release);

	// Setpoint timer
	setpoint_timer = nh.createTimer(ros::Duration(1 / nh_priv.param("setpoint_rate", 30.0)), &publishSetpoint, false, false);

	position_msg.header.frame_id = local_frame;
	position_raw_msg.header.frame_id = local_frame;
	position_raw_msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
	//rates_msg.header.frame_id = fcu_frame;

	ROS_INFO("ready");
	ros::spin();
}
