/*
 * VPE publisher node
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <string>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Trigger.h>
// #include <aruco_pose/MarkerArray.h>

using std::string;
using namespace geometry_msgs;

bool reset_flag = true; // offset should be reset on the start
string local_frame_id, frame_id, child_frame_id, offset_frame_id;
tf2_ros::Buffer tf_buffer;
ros::Publisher vpe_pub;
ros::Subscriber local_position_sub;
ros::Timer zero_timer;
PoseStamped vpe, pose;
ros::Time got_local_pos(0);
ros::Duration publish_zero_timeout, publish_zero_duration, offset_timeout;
TransformStamped offset;

void publishZero(const ros::TimerEvent& e)
{
	if (!vpe.header.stamp.isZero() && e.current_real - vpe.header.stamp < publish_zero_timeout) return; // have vpe

	if (!pose.header.stamp.isZero() && e.current_real - pose.header.stamp < publish_zero_timeout) { // have local position
		if (got_local_pos.isZero()) {
			ROS_INFO("got local position");
			got_local_pos = e.current_real;
		}

		if (e.current_real - got_local_pos > publish_zero_duration) return; // stop publishing zero
	} else {
		// lost local position
		got_local_pos = ros::Time(0);
	}

	ROS_INFO_THROTTLE(10, "publish zero");
	geometry_msgs::PoseStamped zero;
	zero.header.frame_id = local_frame_id;
	zero.header.stamp = e.current_real;
	zero.pose.orientation.w = 1;
	vpe_pub.publish(zero);
}

void localPositionCallback(const PoseStamped& msg) { pose = msg; }

inline Pose getPose(const PoseStampedConstPtr& pose) { return pose->pose; }

inline Pose getPose(const PoseWithCovarianceStampedConstPtr& pose) { return pose->pose.pose; }

inline void keepYaw(Quaternion& quaternion)
{
	tf::Quaternion q;
	q.setRPY(0, 0, tf::getYaw(quaternion));
	tf::quaternionTFToMsg(q, quaternion);
}

template <typename T>
void callback(const T& msg)
{
	static tf2_ros::StaticTransformBroadcaster br;

	try {
		if (!frame_id.empty()) {
			// get VPE transform from TF
			auto transform = tf_buffer.lookupTransform(frame_id, child_frame_id,
													msg->header.stamp, ros::Duration(0.02));
			vpe.pose.position.x = transform.transform.translation.x;
			vpe.pose.position.y = transform.transform.translation.y;
			vpe.pose.position.z = transform.transform.translation.z;
			vpe.pose.orientation = transform.transform.rotation;
		} else {
			vpe.pose = getPose(msg);
		}

		// offset
		if (!offset_frame_id.empty()) {
			if (reset_flag || msg->header.stamp - vpe.header.stamp > offset_timeout) {
				// calculate the offset
				if (!frame_id.empty()) {
					// calculate from TF
					offset = tf_buffer.lookupTransform(local_frame_id, frame_id,
					                                   msg->header.stamp, ros::Duration(0.02));
					// offset.header.frame_id = vpe.header.frame_id;
					offset.child_frame_id = offset_frame_id;

				} else {
					// calculate transform between pose in vpe frame and pose in local frame
					TransformStamped local_pose = tf_buffer.lookupTransform(local_frame_id, child_frame_id,
					                                                        msg->header.stamp, ros::Duration(0.02));
					keepYaw(local_pose.transform.rotation);

					tf::Transform vpeTransform, poseTransform;
					tf::poseMsgToTF(vpe.pose, vpeTransform);
					tf::transformMsgToTF(local_pose.transform, poseTransform);
					tf::Transform offset_tf = vpeTransform.inverseTimes(poseTransform);
					tf::transformTFToMsg(offset_tf, offset.transform);
					offset.header.frame_id = local_frame_id;
					offset.header.stamp = msg->header.stamp;
					offset.child_frame_id = offset_frame_id;
				}

				br.sendTransform(offset);
				reset_flag = false;
				ROS_INFO("offset reset");
			}
			// apply the offset
			tf2::doTransform(vpe, vpe, offset);
		}

		vpe.header.frame_id = local_frame_id;
		vpe.header.stamp = msg->header.stamp;
		vpe_pub.publish(vpe);

	} catch (const tf2::TransformException& e) {
		ROS_WARN_THROTTLE(5, "%s", e.what());
	}
}

bool reset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
	reset_flag = true;
	res.success = true;
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "vpe_publisher");
	ros::NodeHandle nh, nh_priv("~");

	tf2_ros::TransformListener tf_listener(tf_buffer);

	nh_priv.param<string>("frame_id", frame_id, ""); // name for used visual pose frame
	nh_priv.param<string>("offset_frame_id", offset_frame_id, ""); // name for published offset frame

	nh.param<string>("mavros/local_position/frame_id", local_frame_id, "map");
	nh.param<string>("mavros/local_position/tf/child_frame_id", child_frame_id, "base_link");
	offset_timeout = ros::Duration(nh_priv.param("offset_timeout", 3.0));

	if (!frame_id.empty()) {
		ROS_INFO("using data from TF");
	} else {
		ROS_INFO("using data topic");
	}

	auto pose_sub = nh_priv.subscribe<PoseStamped>("pose", 1, &callback);
	auto pose_cov_sub = nh_priv.subscribe<PoseWithCovarianceStamped>("pose_cov", 1, &callback);
	//auto markers_sub = nh_priv.subscribe<aruco_pose::MarkerArray>("markers", 1, &callback);

	vpe_pub = nh_priv.advertise<PoseStamped>("vpe", 1);
	//vpe_cov_pub = nh_priv_.advertise<PoseStamped>("pose_cov_pub", 1);

	if (nh_priv.param("force_init", false) || nh_priv.param("publish_zero", false)) { // publish_zero is old name
		// publish zero to initialize the local position
		zero_timer = nh.createTimer(ros::Duration(0.1), &publishZero);
		publish_zero_timeout = ros::Duration(nh_priv.param("force_init_timeout", 5.0));
		publish_zero_duration = ros::Duration(nh_priv.param("force_init_duration", 5.0));
		local_position_sub = nh.subscribe("mavros/local_position/pose", 1, &localPositionCallback);
	}

	auto reset_serv = nh_priv.advertiseService("reset", &reset);

	ROS_INFO("ready");
	ros::spin();
}
