/*
 * Detecting and pose estimation of ArUco markers
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

/*
 * Code is based on https://github.com/UbiquityRobotics/fiducials, which is distributed
 * under the BSD license.
 */

#include <math.h>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <aruco_pose/Marker.h>
#include <aruco_pose/MarkerArray.h>
#include <aruco_pose/DetectorConfig.h>
#include <aruco_pose/SetMarkers.h>

#include "draw.h"
#include "utils.h"
#include <memory>
#include <functional>

using std::vector;
using cv::Mat;

class ArucoDetect : public nodelet::Nodelet {
private:
	std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
	std::shared_ptr<dynamic_reconfigure::Server<aruco_pose::DetectorConfig>> dyn_srv_;
	bool enabled_ = true;
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	cv::Ptr<cv::aruco::DetectorParameters> parameters_;
	image_transport::Publisher debug_pub_;
	image_transport::CameraSubscriber img_sub_;
	ros::Publisher markers_pub_, vis_markers_pub_;
	ros::Subscriber map_markers_sub_;
	ros::ServiceServer set_markers_srv_;
	bool estimate_poses_, send_tf_, flip_vertical_, auto_flip_, use_map_markers_;
	bool waiting_for_map_;
	double length_;
	ros::Duration transform_timeout_;
	std::unordered_map<int, double> length_override_;
	std::string frame_id_prefix_, known_vertical_;
	Mat camera_matrix_, dist_coeffs_;
	aruco_pose::MarkerArray array_;
	std::unordered_set<int> map_markers_ids_;
	visualization_msgs::MarkerArray vis_array_;

public:
	virtual void onInit()
	{
		ros::NodeHandle& nh_ = getNodeHandle();
		ros::NodeHandle& nh_priv_ = getPrivateNodeHandle();

		br_.reset(new tf2_ros::TransformBroadcaster());
		tf_buffer_.reset(new tf2_ros::Buffer());
		tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_, nh_));

		int dictionary;
		dictionary = nh_priv_.param("dictionary", 2);
		estimate_poses_ = nh_priv_.param("estimate_poses", true);
		send_tf_ = nh_priv_.param("send_tf", true);
		use_map_markers_ = nh_priv_.param("use_map_markers", false);
		waiting_for_map_ = use_map_markers_;
		if (estimate_poses_ && !nh_priv_.getParam("length", length_)) {
			NODELET_FATAL("can't estimate marker's poses as ~length parameter is not defined");
			ros::shutdown();
		}
		readLengthOverride(nh_priv_);
		transform_timeout_ = ros::Duration(nh_priv_.param("transform_timeout", 0.02));

		known_vertical_ = nh_priv_.param("known_vertical", nh_priv_.param("known_tilt", std::string(""))); // known_tilt is an old name
		flip_vertical_ = nh_priv_.param<bool>("flip_vertical", false);
		auto_flip_ = nh_priv_.param("auto_flip", false);

		frame_id_prefix_ = nh_priv_.param<std::string>("frame_id_prefix", "aruco_");

		camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

		dictionary_ = cv::aruco::getPredefinedDictionary(static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary));
		parameters_ = cv::aruco::DetectorParameters::create();

		image_transport::ImageTransport it(nh_);
		image_transport::ImageTransport it_priv(nh_priv_);

		dyn_srv_ = std::make_shared<dynamic_reconfigure::Server<aruco_pose::DetectorConfig>>(nh_priv_);
		dyn_srv_->setCallback(std::bind(&ArucoDetect::paramCallback, this, std::placeholders::_1, std::placeholders::_2));

		set_markers_srv_ = nh_priv_.advertiseService("set_length_override", &ArucoDetect::setMarkers, this);

		debug_pub_ = it_priv.advertise("debug", 1);
		markers_pub_ = nh_priv_.advertise<aruco_pose::MarkerArray>("markers", 1);
		vis_markers_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray>("visualization", 1);
		img_sub_ = it.subscribeCamera("image_raw", 1, &ArucoDetect::imageCallback, this);
		map_markers_sub_ = nh_.subscribe("map_markers", 1, &ArucoDetect::mapMarkersCallback, this);

		NODELET_INFO("ready");
	}

private:
	void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &cinfo)
	{
		if (!enabled_) return;
		if (waiting_for_map_) return;

		Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

		vector<int> ids;
		vector<vector<cv::Point2f>> corners, rejected;
		vector<cv::Vec3d> rvecs, tvecs;
		vector<cv::Point3f> obj_points;
		geometry_msgs::TransformStamped vertical;

		// Detect markers
		cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters_, rejected);

		array_.header.stamp = msg->header.stamp;
		array_.header.frame_id = msg->header.frame_id;
		array_.markers.clear();

		if (ids.size() != 0) {
			parseCameraInfo(cinfo, camera_matrix_, dist_coeffs_);

			// Estimate individual markers' poses
			if (estimate_poses_) {
				cv::aruco::estimatePoseSingleMarkers(corners, length_, camera_matrix_, dist_coeffs_,
				                                     rvecs, tvecs);

				// process length override, TODO: efficiency
				if (!length_override_.empty()) {
					for (unsigned int i = 0; i < ids.size(); i++) {
						int id = ids[i];
						auto item = length_override_.find(id);
						if (item != length_override_.end()) { // found override
							vector<cv::Vec3d> rvecs_current, tvecs_current;
							vector<vector<cv::Point2f>> corners_current;
							corners_current.push_back(corners[i]);
							cv::aruco::estimatePoseSingleMarkers(corners_current, item->second,
							                                     camera_matrix_, dist_coeffs_,
										                         rvecs_current, tvecs_current);
							rvecs[i] = rvecs_current[0];
							tvecs[i] = tvecs_current[0];
						}
					}
				}

				if (!known_vertical_.empty()) {
					try {
						vertical = tf_buffer_->lookupTransform(msg->header.frame_id, known_vertical_,
						                                       msg->header.stamp, transform_timeout_);
					} catch (const tf2::TransformException& e) {
						NODELET_WARN_THROTTLE(5, "can't retrieve known vertical: %s", e.what());
					}
				}
			}

			array_.markers.reserve(ids.size());
			aruco_pose::Marker marker;
			vector<geometry_msgs::TransformStamped> transforms;
			transforms.reserve(ids.size());
			geometry_msgs::TransformStamped transform;
			transform.header.stamp = msg->header.stamp;
			transform.header.frame_id = msg->header.frame_id;

			for (unsigned int i = 0; i < ids.size(); i++) {
				marker.id = ids[i];
				marker.length = getMarkerLength(marker.id);
				fillCorners(marker, corners[i]);

				if (estimate_poses_) {
					fillPose(marker.pose, rvecs[i], tvecs[i]);

					// apply known vertical (if enabled and vertical frame available)
					if (!known_vertical_.empty() && !vertical.header.frame_id.empty()) {
						applyVertical(marker.pose.orientation, vertical.transform.rotation, false, auto_flip_);
					}

					if (send_tf_) {
						transform.child_frame_id = getChildFrameId(ids[i]);

						// check if such static transform is in the map
						if (map_markers_ids_.find(ids[i]) == map_markers_ids_.end()) {
							// check if a markers with that id is already added
							bool send = true;
							for (auto &t : transforms) {
								if (t.child_frame_id == transform.child_frame_id) {
									send = false;
									break;
								}
							}
							if (send) {
								transform.transform.rotation = marker.pose.orientation;
								fillTranslation(transform.transform.translation, tvecs[i]);
								transforms.push_back(transform);
							}
						}
					}
				}
				array_.markers.push_back(marker);
			}

			if (send_tf_) {
				br_->sendTransform(transforms);
			}
		}

		markers_pub_.publish(array_);

		// Publish visualization markers
		if (estimate_poses_ && vis_markers_pub_.getNumSubscribers() != 0) {
			// Delete all markers
			visualization_msgs::Marker vis_marker;
			vis_marker.action = visualization_msgs::Marker::DELETEALL;
			vis_array_.markers.clear();
			vis_array_.markers.reserve(ids.size() + 1);
			vis_array_.markers.push_back(vis_marker);

			for (unsigned int i = 0; i < ids.size(); i++)
				pushVisMarkers(msg->header.frame_id, msg->header.stamp, array_.markers[i].pose,
				               getMarkerLength(ids[i]), ids[i], i);

			vis_markers_pub_.publish(vis_array_);
		}

		// Publish debug image
		if (debug_pub_.getNumSubscribers() != 0) {
			Mat debug = image.clone();
			cv::aruco::drawDetectedMarkers(debug, corners, ids); // draw markers
			if (estimate_poses_)
				for (unsigned int i = 0; i < ids.size(); i++)
					_drawAxis(debug, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], getMarkerLength(ids[i]));

			cv_bridge::CvImage out_msg;
			out_msg.header.frame_id = msg->header.frame_id;
			out_msg.header.stamp = msg->header.stamp;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			out_msg.image = debug;
			debug_pub_.publish(out_msg.toImageMsg());
		}
	}

	inline void fillCorners(aruco_pose::Marker& marker, const vector<cv::Point2f>& corners) const
	{
		marker.c1.x = corners[0].x;
		marker.c2.x = corners[1].x;
		marker.c3.x = corners[2].x;
		marker.c4.x = corners[3].x;
		marker.c1.y = corners[0].y;
		marker.c2.y = corners[1].y;
		marker.c3.y = corners[2].y;
		marker.c4.y = corners[3].y;
	}

	inline void fillPose(geometry_msgs::Pose& pose, const cv::Vec3d& rvec, const cv::Vec3d& tvec) const
	{
		pose.position.x = tvec[0];
		pose.position.y = tvec[1];
		pose.position.z = tvec[2];

		double angle = norm(rvec);
		cv::Vec3d axis = rvec / angle;

		tf2::Quaternion q;
		q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

		pose.orientation.w = q.w();
		pose.orientation.x = q.x();
		pose.orientation.y = q.y();
		pose.orientation.z = q.z();
	}

	inline void fillTranslation(geometry_msgs::Vector3& translation, const cv::Vec3d& tvec) const
	{
		translation.x = tvec[0];
		translation.y = tvec[1];
		translation.z = tvec[2];
	}

	void pushVisMarkers(const std::string& frame_id, const ros::Time& stamp,
	                    const geometry_msgs::Pose &pose, double length, int id, int index)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = frame_id;
		marker.header.stamp = stamp;
		marker.action = visualization_msgs::Marker::ADD;
		marker.id = index;

		// Marker
		marker.ns = "aruco_marker";
		marker.type = visualization_msgs::Marker::CUBE;
		marker.scale.x = length;
		marker.scale.y = length;
		marker.scale.z = 0.001;
		marker.color.r = 1;
		marker.color.g = 1;
		marker.color.b = 1;
		marker.color.a = 0.9;
		marker.pose = pose;
		vis_array_.markers.push_back(marker);

		// Label
		marker.ns = "aruco_marker_label";
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.scale.z = length * 0.6;
		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 0;
		marker.color.a = 1;
		marker.text = std::to_string(id);
		marker.pose = pose;
		vis_array_.markers.push_back(marker);
	}

	inline std::string getChildFrameId(int id) const
	{
		return frame_id_prefix_ + std::to_string(id);
	}

	void readLengthOverride(ros::NodeHandle& nh)
	{
		std::map<std::string, double> length_override;
		nh.getParam("length_override", length_override);
		for (auto const& item : length_override) {
			length_override_[std::stoi(item.first)] = item.second;
		}
	}

	inline double getMarkerLength(int id)
	{
		auto item = length_override_.find(id);
		if (item != length_override_.end()) {
			return item->second;
		} else {
			return length_;
		}
	}

	bool setMarkers(aruco_pose::SetMarkers::Request& req, aruco_pose::SetMarkers::Response& res)
	{
		for (auto const& marker : req.markers) {
			if (marker.id > 999) {
				res.message = "Invalid marker id: " + std::to_string(marker.id);
				ROS_ERROR("%s", res.message.c_str());
				return true;
			}
			if (!std::isfinite(marker.length) || marker.length <= 0) {
				res.message = "Invalid marker " + std::to_string(marker.id) + " length: " + std::to_string(marker.length);
				ROS_ERROR("%s", res.message.c_str());
				return true;
			}
		}

		for (auto const& marker : req.markers) {
			length_override_[marker.id] = marker.length;
		}

		res.success = true;
		return true;
	}

	void mapMarkersCallback(const aruco_pose::MarkerArray& msg)
	{
		map_markers_ids_.clear();
		for (auto const& marker : msg.markers) {
			map_markers_ids_.insert(marker.id);
			if (use_map_markers_) {
				if (length_override_.find(marker.id) == length_override_.end()) {
					length_override_[marker.id] = marker.length;
				}
			}
		}
		waiting_for_map_ = false;
	}

	void paramCallback(aruco_pose::DetectorConfig &config, uint32_t level)
	{
		enabled_ = config.enabled && config.length > 0;
		length_ = config.length;
		parameters_->adaptiveThreshConstant = config.adaptiveThreshConstant;
		parameters_->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
		parameters_->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
		parameters_->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
		parameters_->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
		parameters_->cornerRefinementMethod = config.cornerRefinementMethod;
		parameters_->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
		parameters_->cornerRefinementWinSize = config.cornerRefinementWinSize;
#if ((CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 4) && (CV_VERSION_REVISION >= 7)) || (CV_VERSION_MAJOR > 3)
		parameters_->detectInvertedMarker = config.detectInvertedMarker;
#endif
		parameters_->errorCorrectionRate = config.errorCorrectionRate;
		parameters_->minCornerDistanceRate = config.minCornerDistanceRate;
		parameters_->markerBorderBits = config.markerBorderBits;
		parameters_->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
		parameters_->minDistanceToBorder = config.minDistanceToBorder;
		parameters_->minMarkerDistanceRate = config.minMarkerDistanceRate;
		parameters_->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
		parameters_->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
		parameters_->minOtsuStdDev = config.minOtsuStdDev;
		parameters_->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
		parameters_->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
		parameters_->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
#if ((CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 4) && (CV_VERSION_REVISION >= 2)) || (CV_VERSION_MAJOR > 3)
		parameters_->aprilTagQuadDecimate = config.aprilTagQuadDecimate;
		parameters_->aprilTagQuadSigma = config.aprilTagQuadSigma;
#endif
	}
};

PLUGINLIB_EXPORT_CLASS(ArucoDetect, nodelet::Nodelet)
