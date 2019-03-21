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

#include "utils.h"

using std::vector;
using cv::Mat;

class ArucoDetect : public nodelet::Nodelet {
private:
	ros::NodeHandle nh_, nh_priv_;
	tf2_ros::TransformBroadcaster br_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_{tf_buffer_};
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	cv::Ptr<cv::aruco::DetectorParameters> parameters_;
	image_transport::Publisher debug_pub_;
	image_transport::CameraSubscriber img_sub_;
	ros::Publisher markers_pub_, vis_markers_pub_;
	bool estimate_poses_, send_tf_, auto_flip_;
	double length_;
	std::unordered_map<int, double> length_override_;
	std::string frame_id_prefix_, known_tilt_;
	Mat camera_matrix_, dist_coeffs_;
	aruco_pose::MarkerArray array_;
	visualization_msgs::MarkerArray vis_array_;

public:
	virtual void onInit()
	{
		nh_ = getNodeHandle();
		nh_priv_ = getPrivateNodeHandle();

		int dictionary;
		nh_priv_.param("dictionary", dictionary, 2);
		nh_priv_.param("estimate_poses", estimate_poses_, true);
		nh_priv_.param("send_tf", send_tf_, true);
		if (estimate_poses_ && !nh_priv_.getParam("length", length_)) {
			ROS_FATAL("aruco_detect: can't estimate marker's poses as ~length parameter is not defined");
			ros::shutdown();
		}
		readLengthOverride();

		nh_priv_.param<std::string>("known_tilt", known_tilt_, "");
		nh_priv_.param("auto_flip", auto_flip_, false);

		nh_priv_.param<std::string>("frame_id_prefix", frame_id_prefix_, "aruco_");

		camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
		dist_coeffs_ = cv::Mat::zeros(8, 1, CV_64F);

		dictionary_ = cv::aruco::getPredefinedDictionary(static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary));
		parameters_ = cv::aruco::DetectorParameters::create();
		parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

		image_transport::ImageTransport it(nh_);
		image_transport::ImageTransport it_priv(nh_priv_);

		debug_pub_ = it_priv.advertise("debug", 1);
		markers_pub_ = nh_priv_.advertise<aruco_pose::MarkerArray>("markers", 1);
		vis_markers_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray>("visualization", 1);
		img_sub_ = it.subscribeCamera("image_raw", 1, &ArucoDetect::imageCallback, this);

		ROS_INFO("aruco_detect: ready");
	}

private:
	void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr &cinfo)
	{
		Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

		vector<int> ids;
		vector<vector<cv::Point2f>> corners, rejected;
		vector<cv::Vec3d> rvecs, tvecs;
		vector<cv::Point3f> obj_points;
		geometry_msgs::TransformStamped snap_to;

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

				if (!known_tilt_.empty()) {
					try {
						snap_to = tf_buffer_.lookupTransform(msg->header.frame_id, known_tilt_,
						                                     msg->header.stamp, ros::Duration(0.02));
					} catch (const tf2::TransformException& e) {
						ROS_WARN_THROTTLE(5, "aruco_detect: can't snap: %s", e.what());
					}
				}
			}

			array_.markers.reserve(ids.size());
			aruco_pose::Marker marker;
			geometry_msgs::TransformStamped transform;
			transform.header.stamp = msg->header.stamp;
			transform.header.frame_id = msg->header.frame_id;

			for (unsigned int i = 0; i < ids.size(); i++) {
				marker.id = ids[i];
				marker.length = getMarkerLength(marker.id);
				fillCorners(marker, corners[i]);

				if (estimate_poses_) {
					fillPose(marker.pose, rvecs[i], tvecs[i]);

					// snap orientation (if enabled and snap frame available)
					if (!known_tilt_.empty() && !snap_to.header.frame_id.empty()) {
						snapOrientation(marker.pose.orientation, snap_to.transform.rotation, auto_flip_);
					}

					// TODO: check IDs are unique
					if (send_tf_) {
						transform.child_frame_id = getChildFrameId(ids[i]);
						transform.transform.rotation = marker.pose.orientation;
						fillTranslation(transform.transform.translation, tvecs[i]);
						br_.sendTransform(transform);
					}
				}
				array_.markers.push_back(marker);
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
					cv::aruco::drawAxis(debug, camera_matrix_, dist_coeffs_,
					                    rvecs[i], tvecs[i], getMarkerLength(ids[i]));

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

	void readLengthOverride()
	{
		std::map<std::string, double> length_override;
		nh_priv_.getParam("length_override", length_override);
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
};

PLUGINLIB_EXPORT_CLASS(ArucoDetect, nodelet::Nodelet)
