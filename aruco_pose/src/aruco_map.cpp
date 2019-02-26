/*
 * Detecting and pose estimation of ArUco markers maps
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
#include <string>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <aruco_pose/MarkerArray.h>
#include <aruco_pose/Marker.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "draw.h"
#include "utils.h"

using std::vector;
using cv::Mat;

class ArucoMap : public nodelet::Nodelet {
private:
	ros::NodeHandle nh_, nh_priv_;
	ros::Publisher img_pub_, pose_pub_, vis_markers_pub_;
	ros::Subscriber markers_sub_, cinfo_sub;
	cv::Ptr<cv::aruco::Board> board_;
	Mat camera_matrix_, dist_coeffs_;
	geometry_msgs::TransformStamped transform_;
	geometry_msgs::PoseWithCovarianceStamped pose_;
	tf2_ros::TransformBroadcaster br_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_{tf_buffer_};
	visualization_msgs::MarkerArray vis_array_;
	std::string known_tilt_;
	int image_width_, image_height_, image_margin_;
	bool has_camera_info_ = false;

public:
	virtual void onInit()
	{
		nh_ = getNodeHandle();
		nh_priv_ = getPrivateNodeHandle();

		image_transport::ImageTransport it_priv(nh_priv_);

		// TODO: why image_transport doesn't work here?
		img_pub_ = nh_priv_.advertise<sensor_msgs::Image>("image", 1, true);

		board_ = cv::makePtr<cv::aruco::Board>();
		board_->dictionary = cv::aruco::getPredefinedDictionary(
			                 static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(nh_priv_.param("dictionary", 2)));
		camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
		dist_coeffs_ = cv::Mat::zeros(8, 1, CV_64F);

		std::string type, map, map_name;
		nh_priv_.param<std::string>("type", type, "map");
		nh_priv_.param<std::string>("name", map_name, "map");
		nh_priv_.param<std::string>("frame_id", transform_.child_frame_id, "aruco_map");
		nh_priv_.param<std::string>("known_tilt", known_tilt_, "");
		nh_priv_.param("image_width", image_width_, 2000);
		nh_priv_.param("image_height", image_height_, 2000);
		nh_priv_.param("image_margin", image_margin_, 200);

		// createStripLine();

		if (type == "map") {
			param(nh_priv_, "map", map);
			loadMap(map);
		} else if (type == "gridboard") {
			createGridBoard();
		} else {
			ROS_FATAL("aruco_map: unknown type: %s", type.c_str());
			ros::shutdown();
		}

		pose_pub_ = nh_priv_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
		vis_markers_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);

		// TODO: use synchronised subscriber
		markers_sub_ = nh_.subscribe("markers", 1, &ArucoMap::markersCallback, this);
		cinfo_sub = nh_.subscribe("camera_info", 1, &ArucoMap::cinfoCallback, this);

		publishMapImage();
		vis_markers_pub_.publish(vis_array_);

		ROS_INFO("aruco_map: ready");
	}

	void markersCallback(const aruco_pose::MarkerArray& markers)
	{
		if (!has_camera_info_) return;
		if (markers.markers.empty()) return;

		int count = markers.markers.size();
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		ids.reserve(count);
		corners.reserve(count);

		for(auto const &marker : markers.markers) {
			ids.push_back(marker.id);
			std::vector<cv::Point2f> marker_corners = {
				cv::Point2f(marker.c1.x, marker.c1.y),
				cv::Point2f(marker.c2.x, marker.c2.y),
				cv::Point2f(marker.c3.x, marker.c3.y),
				cv::Point2f(marker.c4.x, marker.c4.y)
			};
			corners.push_back(marker_corners);
		}

		Mat obj_points, img_points;
		cv::Vec3d rvec, tvec;

		if (known_tilt_.empty()) {
			// simple estimation
			int valid = cv::aruco::estimatePoseBoard(corners, ids, board_, camera_matrix_, dist_coeffs_,
		                                             rvec, tvec, false);
			if (!valid) return;

			transform_.header.stamp = markers.header.stamp;
			transform_.header.frame_id = markers.header.frame_id;
			pose_.header = transform_.header;
			fillPose(pose_.pose.pose, rvec, tvec);
			fillTransform(transform_.transform, rvec, tvec);

		} else {
			// estimation with "snapping"
			cv::aruco::getBoardObjectAndImagePoints(board_, corners, ids, obj_points, img_points);
			if (obj_points.empty()) return;

			double center_x = 0, center_y = 0;
			alignObjPointsToCenter(obj_points, center_x, center_y);

			int res = solvePnP(obj_points, img_points, camera_matrix_, dist_coeffs_, rvec, tvec, false);
			if (!res) return;

			fillTransform(transform_.transform, rvec, tvec);
			try {
				geometry_msgs::TransformStamped snap_to = tf_buffer_.lookupTransform(markers.header.frame_id,
				                                          known_tilt_, markers.header.stamp, ros::Duration(0.02));
				snapOrientation(transform_.transform.rotation, snap_to.transform.rotation);
			} catch (const tf2::TransformException& e) {
				ROS_WARN_THROTTLE(1, "aruco_map: can't snap: %s", e.what());
			}

			geometry_msgs::TransformStamped shift;
			shift.transform.translation.x = -center_x;
			shift.transform.translation.y = -center_y;
			shift.transform.rotation.w = 1;
			tf2::doTransform(shift, transform_, transform_);

			transform_.header.stamp = markers.header.stamp;
			transform_.header.frame_id = markers.header.frame_id;
			pose_.header = transform_.header;
			transformToPose(transform_.transform, pose_.pose.pose);
		}

		if (!transform_.child_frame_id.empty()) {
			br_.sendTransform(transform_);
		}
		pose_pub_.publish(pose_);
	}

	void cinfoCallback(const sensor_msgs::CameraInfoConstPtr& cinfo)
	{
		parseCameraInfo(cinfo, camera_matrix_, dist_coeffs_);
		has_camera_info_ = true;
	}

	void alignObjPointsToCenter(Mat &obj_points, double &center_x, double &center_y) const
	{
		// Align object points to the center of mass
		double sum_x = 0;
		double sum_y = 0;

		for (int i = 0; i < obj_points.rows; i++) 		{
			sum_x += obj_points.at<float>(i, 0);
			sum_y += obj_points.at<float>(i, 1);
		}

		center_x = sum_x / obj_points.rows;
		center_y = sum_y / obj_points.rows;

		for (int i = 0; i < obj_points.rows; i++) 		{
			obj_points.at<float>(i, 0) -= center_x;
			obj_points.at<float>(i, 1) -= center_y;
		}
	}

	void loadMap(std::string filename)
	{
		std::ifstream f(filename);
		std::string line;

		if (!f.good()) {
			ROS_FATAL("aruco_map: %s - %s", strerror(errno), filename.c_str());
			ros::shutdown();
		}

		while (std::getline(f, line)) {
			int id;
			double length, x, y, z, yaw, pitch, roll;

			std::istringstream s(line);

			if (!(s >> id >> length >> x >> y >> z >> yaw >> pitch >> roll)) {
				ROS_ERROR("aruco_map: cannot parse line: %s", line.c_str());
				continue;
			}
			addMarker(id, length, x, y, z, yaw, pitch, roll);
		}

		ROS_INFO("aruco_map: loading %s complete (%d markers)", filename.c_str(), static_cast<int>(board_->ids.size()));
	}

	void createGridBoard()
	{
		ROS_INFO("aruco_map: generate gridboard");
		ROS_WARN("aruco_map: gridboard maps are deprecated");

		int markers_x, markers_y, first_marker;
		double markers_side, markers_sep_x, markers_sep_y;
		std::vector<int> marker_ids;
		nh_priv_.param<int>("markers_x", markers_x, 10);
		nh_priv_.param<int>("markers_y", markers_y, 10);
		nh_priv_.param<int>("first_marker", first_marker, 0);

		param(nh_priv_, "markers_side", markers_side);
		param(nh_priv_, "markers_sep_x", markers_sep_x);
		param(nh_priv_, "markers_sep_y", markers_sep_y);

		if (nh_priv_.getParam("marker_ids", marker_ids)) {
			if ((unsigned int)(markers_x * markers_y) != marker_ids.size()) {
				ROS_FATAL("~marker_ids length should be equal to ~markers_x * ~markers_y");
				ros::shutdown();
			}
		} else {
			// Fill marker_ids automatically
			marker_ids.resize(markers_x * markers_y);
			for (int i = 0; i < markers_x * markers_y; i++)
			{
				marker_ids.at(i) = first_marker++;
			}
		}

		double max_y = markers_y * markers_side + (markers_y - 1) * markers_sep_y;
		for(int y = 0; y < markers_y; y++) {
			for(int x = 0; x < markers_x; x++) {
				double x_pos = x * (markers_side + markers_sep_x);
				double y_pos = max_y - y * (markers_side + markers_sep_y) - markers_side;
				ROS_INFO("add marker %d %g %g", marker_ids[y * markers_y + x], x_pos, y_pos);
				addMarker(marker_ids[y * markers_y + x], markers_side, x_pos, y_pos, 0, 0, 0, 0);
			}
		}
	}

	// void createStripLine()
	// {
	// 	visualization_msgs::Marker marker;
	// 	marker.header.frame_id = transform_.child_frame_id;
	// 	marker.action = visualization_msgs::Marker::ADD;
	// 	marker.ns = "aruco_map_link";
	// 	marker.type = visualization_msgs::Marker::LINE_STRIP;
	// 	marker.scale.x = 0.02;
	// 	marker.color.g = 1;
	// 	marker.color.a = 0.8;
	// 	marker.frame_locked = true;
	// 	marker.pose.orientation.w = 1;
	// 	vis_array_.markers.push_back(marker);
	// }

	void addMarker(int id, double length, double x, double y, double z,
				   double yaw, double pitch, double roll)
	{
		// Create transform
		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		tf::Transform transform(q, tf::Vector3(x, y, z));

		/* marker's corners:
			0    1
			3    2
		*/
		double halflen = length / 2;
		tf::Point p0(-halflen, halflen, 0);
		tf::Point p1(halflen, halflen, 0);
		tf::Point p2(halflen, -halflen, 0);
		tf::Point p3(-halflen, -halflen, 0);
		p0 = transform * p0;
		p1 = transform * p1;
		p2 = transform * p2;
		p3 = transform * p3;

		vector<cv::Point3f> obj_points = {
			cv::Point3f(p0.x(), p0.y(), p0.z()),
			cv::Point3f(p1.x(), p1.y(), p1.z()),
			cv::Point3f(p2.x(), p2.y(), p2.z()),
			cv::Point3f(p3.x(), p3.y(), p3.z())
		};

		board_->ids.push_back(id);
		board_->objPoints.push_back(obj_points);

		// Add visualization marker
		visualization_msgs::Marker marker;
		marker.header.frame_id = transform_.child_frame_id;
		// marker.header.stamp = stamp;
		marker.action = visualization_msgs::Marker::ADD;
		marker.id = vis_array_.markers.size();
		marker.ns = "aruco_map_marker";
		marker.type = visualization_msgs::Marker::CUBE;
		marker.scale.x = length;
		marker.scale.y = length;
		marker.scale.z = 0.001;
		marker.color.r = 1;
		marker.color.g = 0.5;
		marker.color.b = 0.5;
		marker.color.a = 0.8;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		tf::quaternionTFToMsg(q, marker.pose.orientation);
		marker.frame_locked = true;
		vis_array_.markers.push_back(marker);

		// Add linking line
		// geometry_msgs::Point p;
		// p.x = x;
		// p.y = y;
		// p.z = z;
		// vis_array_.markers.at(0).points.push_back(p);
	}

	void publishMapImage()
	{
		cv::Mat image;
		cv_bridge::CvImage msg;
		drawPlanarBoard(board_, cv::Size(image_width_, image_height_), image, image_margin_, 1);

		cv::cvtColor(image, image, CV_GRAY2BGR);
		msg.encoding = sensor_msgs::image_encodings::BGR8;
		msg.image = image;
		img_pub_.publish(msg.toImageMsg());
	}
};

PLUGINLIB_EXPORT_CLASS(ArucoMap, nodelet::Nodelet)
