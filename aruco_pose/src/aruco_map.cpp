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
#include <algorithm>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
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
using sensor_msgs::Image;
using sensor_msgs::CameraInfo;
using aruco_pose::MarkerArray;

typedef message_filters::sync_policies::ExactTime<Image, CameraInfo, MarkerArray> SyncPolicy;

class ArucoMap : public nodelet::Nodelet {
private:
	ros::NodeHandle nh_, nh_priv_;
	ros::Publisher img_pub_, pose_pub_, markers_pub_, vis_markers_pub_;
	image_transport::Publisher debug_pub_;
	message_filters::Subscriber<Image> image_sub_;
	message_filters::Subscriber<CameraInfo> info_sub_;
	message_filters::Subscriber<MarkerArray> markers_sub_;
	boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
	cv::Ptr<cv::aruco::Board> board_;
	Mat camera_matrix_, dist_coeffs_;
	geometry_msgs::TransformStamped transform_;
	geometry_msgs::PoseWithCovarianceStamped pose_;
	vector<geometry_msgs::TransformStamped> markers_transforms_;
	aruco_pose::MarkerArray markers_;
	tf2_ros::TransformBroadcaster br_;
	tf2_ros::StaticTransformBroadcaster static_br_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_{tf_buffer_};
	visualization_msgs::MarkerArray vis_array_;
	std::string known_tilt_, map_, markers_frame_, markers_parent_frame_;
	int image_width_, image_height_, image_margin_;
	bool auto_flip_, image_axis_;

public:
	virtual void onInit()
	{
		nh_ = getNodeHandle();
		nh_priv_ = getPrivateNodeHandle();

		image_transport::ImageTransport it_priv(nh_priv_);

		// TODO: why image_transport doesn't work here?
		img_pub_ = nh_priv_.advertise<sensor_msgs::Image>("image", 1, true);
		markers_pub_ = nh_priv_.advertise<aruco_pose::MarkerArray>("markers", 1, true);

		board_ = cv::makePtr<cv::aruco::Board>();
		board_->dictionary = cv::aruco::getPredefinedDictionary(
			                 static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(nh_priv_.param("dictionary", 2)));
		camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
		dist_coeffs_ = cv::Mat::zeros(8, 1, CV_64F);

		std::string type, map;
		nh_priv_.param<std::string>("type", type, "map");
		nh_priv_.param<std::string>("frame_id", transform_.child_frame_id, "aruco_map");
		nh_priv_.param<std::string>("known_tilt", known_tilt_, "");
		nh_priv_.param("auto_flip", auto_flip_, false);
		nh_priv_.param("image_width", image_width_, 2000);
		nh_priv_.param("image_height", image_height_, 2000);
		nh_priv_.param("image_margin", image_margin_, 200);
		nh_priv_.param("image_axis", image_axis_, true);
		nh_priv_.param<std::string>("markers/frame_id", markers_parent_frame_, transform_.child_frame_id);
		nh_priv_.param<std::string>("markers/child_frame_id_prefix", markers_frame_, "");

		// createStripLine();

		if (type == "map") {
			param(nh_priv_, "map", map);
			loadMap(map);
		} else if (type == "gridboard") {
			createGridBoard();
		} else {
			NODELET_FATAL("unknown type: %s", type.c_str());
			ros::shutdown();
		}

		pose_pub_ = nh_priv_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
		vis_markers_pub_ = nh_priv_.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
		debug_pub_ = it_priv.advertise("debug", 1);

		image_sub_.subscribe(nh_, "image_raw", 1);
		info_sub_.subscribe(nh_, "camera_info", 1);
		markers_sub_.subscribe(nh_, "markers", 1);

		sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), image_sub_, info_sub_, markers_sub_));
		sync_->registerCallback(boost::bind(&ArucoMap::callback, this, _1, _2, _3));

		publishMarkersFrames();
		publishMarkers();
		publishMapImage();
		vis_markers_pub_.publish(vis_array_);

		NODELET_INFO("ready");
	}

	void callback(const sensor_msgs::ImageConstPtr& image,
	              const sensor_msgs::CameraInfoConstPtr& cinfo,
	              const aruco_pose::MarkerArrayConstPtr& markers)
	{
		int valid = 0;
		int count = markers->markers.size();
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::Vec3d rvec, tvec;

		parseCameraInfo(cinfo, camera_matrix_, dist_coeffs_);
		if (markers->markers.empty()) goto publish_debug;

		ids.reserve(count);
		corners.reserve(count);

		for(auto const &marker : markers->markers) {
			ids.push_back(marker.id);
			std::vector<cv::Point2f> marker_corners = {
				cv::Point2f(marker.c1.x, marker.c1.y),
				cv::Point2f(marker.c2.x, marker.c2.y),
				cv::Point2f(marker.c3.x, marker.c3.y),
				cv::Point2f(marker.c4.x, marker.c4.y)
			};
			corners.push_back(marker_corners);
		}

		if (known_tilt_.empty()) {
			// simple estimation
			valid = cv::aruco::estimatePoseBoard(corners, ids, board_, camera_matrix_, dist_coeffs_,
			                                     rvec, tvec, false);
			if (!valid) goto publish_debug;

			transform_.header.stamp = markers->header.stamp;
			transform_.header.frame_id = markers->header.frame_id;
			pose_.header = transform_.header;
			fillPose(pose_.pose.pose, rvec, tvec);
			fillTransform(transform_.transform, rvec, tvec);

		} else {
			Mat obj_points, img_points;
			// estimation with "snapping"
			cv::aruco::getBoardObjectAndImagePoints(board_, corners, ids, obj_points, img_points);
			if (obj_points.empty()) goto publish_debug;

			double center_x = 0, center_y = 0, center_z = 0;
			alignObjPointsToCenter(obj_points, center_x, center_y, center_z);

			valid = solvePnP(obj_points, img_points, camera_matrix_, dist_coeffs_, rvec, tvec, false);
			if (!valid) goto publish_debug;

			fillTransform(transform_.transform, rvec, tvec);
			try {
				geometry_msgs::TransformStamped snap_to = tf_buffer_.lookupTransform(markers->header.frame_id,
				                                          known_tilt_, markers->header.stamp, ros::Duration(0.02));
				snapOrientation(transform_.transform.rotation, snap_to.transform.rotation, auto_flip_);
			} catch (const tf2::TransformException& e) {
				NODELET_WARN_THROTTLE(1, "can't snap: %s", e.what());
			}

			geometry_msgs::TransformStamped shift;
			shift.transform.translation.x = -center_x;
			shift.transform.translation.y = -center_y;
			shift.transform.translation.z = -center_z;
			shift.transform.rotation.w = 1;
			tf2::doTransform(shift, transform_, transform_);

			// for debug topic
			tvec[0] = transform_.transform.translation.x;
			tvec[1] = transform_.transform.translation.y;
			tvec[2] = transform_.transform.translation.z;

			transform_.header.stamp = markers->header.stamp;
			transform_.header.frame_id = markers->header.frame_id;
			pose_.header = transform_.header;
			transformToPose(transform_.transform, pose_.pose.pose);
		}

		if (!transform_.child_frame_id.empty()) {
			br_.sendTransform(transform_);
		}
		pose_pub_.publish(pose_);

publish_debug:
		// publish debug image (even if no map detected)
		if (debug_pub_.getNumSubscribers() > 0) {
			Mat mat = cv_bridge::toCvCopy(image, "bgr8")->image; // copy image as we're planning to modify it
			cv::aruco::drawDetectedMarkers(mat, corners, ids); // draw detected markers
			if (valid) {
				_drawAxis(mat, camera_matrix_, dist_coeffs_, rvec, tvec, 1.0); // draw board axis
			}
			cv_bridge::CvImage out_msg;
			out_msg.header.frame_id = image->header.frame_id;
			out_msg.header.stamp = image->header.stamp;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			out_msg.image = mat;
			debug_pub_.publish(out_msg.toImageMsg());
		}
	}

	void alignObjPointsToCenter(Mat &obj_points, double &center_x, double &center_y, double &center_z) const
	{
		// Align object points to the center of mass
		double sum_x = 0;
		double sum_y = 0;
		double sum_z = 0;

		for (int i = 0; i < obj_points.rows; i++) {
			sum_x += obj_points.at<float>(i, 0);
			sum_y += obj_points.at<float>(i, 1);
			sum_z += obj_points.at<float>(i, 2);
		}

		center_x = sum_x / obj_points.rows;
		center_y = sum_y / obj_points.rows;
		center_z = sum_z / obj_points.rows;

		for (int i = 0; i < obj_points.rows; i++) {
			obj_points.at<float>(i, 0) -= center_x;
			obj_points.at<float>(i, 1) -= center_y;
			obj_points.at<float>(i, 2) -= center_z;
		}
	}

	void loadMap(std::string filename)
	{
		std::ifstream f(filename);
		std::string line;

		if (!f.good()) {
			NODELET_FATAL("%s - %s", strerror(errno), filename.c_str());
			ros::shutdown();
		}

		while (std::getline(f, line)) {
			int id;
			double length, x, y, z, yaw, pitch, roll;

			std::istringstream s(line);

			// Read first character to see whether it's a comment
			char first = 0;
			if (!(s >> first)) {
				// No non-whitespace characters, must be a blank line
				continue;
			}

			if (first == '#') {
				NODELET_DEBUG("Skipping line as a comment: %s", line.c_str());
				continue;
			} else if (isdigit(first)) {
				// Put the digit back into the stream
				// Note that this is a non-modifying putback, so this should work with istreams
				// (see https://en.cppreference.com/w/cpp/io/basic_istream/putback)
				s.putback(first);
			} else {
				// Probably garbage data; inform user and throw an exception, possibly killing nodelet
				NODELET_FATAL("Malformed input: %s", line.c_str());
				ros::shutdown();
				throw std::runtime_error("Malformed input");
			}

			if (!(s >> id >> length >> x >> y)) {
				NODELET_ERROR("Not enough data in line: %s; "
				          "Each marker must have at least id, length, x, y fields", line.c_str());
				continue;
			}
			// Be less strict about z, yaw, pitch roll
			if (!(s >> z)) {
				NODELET_DEBUG("No z coordinate provided for marker %d, assuming 0", id);
				z = 0;
			}
			if (!(s >> yaw)) {
				NODELET_DEBUG("No yaw provided for marker %d, assuming 0", id);
				yaw = 0;
			}
			if (!(s >> pitch)) {
				NODELET_DEBUG("No pitch provided for marker %d, assuming 0", id);
				pitch = 0;
			}
			if (!(s >> roll)) {
				NODELET_DEBUG("No roll provided for marker %d, assuming 0", id);
				roll = 0;
			}
			addMarker(id, length, x, y, z, yaw, pitch, roll);
		}

		NODELET_INFO("loading %s complete (%d markers)", filename.c_str(), static_cast<int>(board_->ids.size()));
	}

	void createGridBoard()
	{
		NODELET_INFO("generate gridboard");
		NODELET_WARN("gridboard maps are deprecated");

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
				NODELET_FATAL("~marker_ids length should be equal to ~markers_x * ~markers_y");
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
				NODELET_INFO("add marker %d %g %g", marker_ids[y * markers_y + x], x_pos, y_pos);
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
		// Check whether the id is in range for current dictionary
		int num_markers = board_->dictionary->bytesList.rows;
		if (num_markers <= id) {
			NODELET_ERROR("Marker id %d is not in dictionary; current dictionary contains %d markers. "
			              "Please see https://github.com/CopterExpress/clever/blob/master/aruco_pose/README.md#parameters for details",
					  id, num_markers);
			return;
		}
		// Check if marker is already in the board
		if (std::count(board_->ids.begin(), board_->ids.end(), id) > 0) {
			NODELET_ERROR("Marker id %d is already in the map", id);
			return;
		}
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

		// Add marker's static transform
		if (!markers_frame_.empty()) {
			geometry_msgs::TransformStamped marker_transform;
			marker_transform.header.frame_id = markers_parent_frame_;
			marker_transform.child_frame_id = markers_frame_ + std::to_string(id);
			tf::transformTFToMsg(transform, marker_transform.transform);
			markers_transforms_.push_back(marker_transform);
		}

		// Add marker to array
		aruco_pose::Marker marker;
		marker.id = id;
		marker.length = length;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		tf::quaternionTFToMsg(q, marker.pose.orientation);
		markers_.markers.push_back(marker);

		// Add visualization marker
		visualization_msgs::Marker vis_marker;
		vis_marker.header.frame_id = transform_.child_frame_id;
		vis_marker.action = visualization_msgs::Marker::ADD;
		vis_marker.id = vis_array_.markers.size();
		vis_marker.ns = "aruco_map_marker";
		vis_marker.type = visualization_msgs::Marker::CUBE;
		vis_marker.scale.x = length;
		vis_marker.scale.y = length;
		vis_marker.scale.z = 0.001;
		vis_marker.color.r = 1;
		vis_marker.color.g = 0.5;
		vis_marker.color.b = 0.5;
		vis_marker.color.a = 0.8;
		vis_marker.pose.position.x = x;
		vis_marker.pose.position.y = y;
		vis_marker.pose.position.z = z;
		tf::quaternionTFToMsg(q, marker.pose.orientation);
		vis_marker.frame_locked = true;
		vis_array_.markers.push_back(vis_marker);

		// Add linking line
		// geometry_msgs::Point p;
		// p.x = x;
		// p.y = y;
		// p.z = z;
		// vis_array_.markers.at(0).points.push_back(p);
	}

	void publishMarkersFrames()
	{
		if (!markers_transforms_.empty()) {
			static_br_.sendTransform(markers_transforms_);
		}
	}

	void publishMarkers()
	{
		markers_pub_.publish(markers_);
	}

	void publishMapImage()
	{
		cv::Size size(image_width_, image_height_);
		cv::Mat image;
		cv_bridge::CvImage msg;

		if (!board_->ids.empty()) {
			_drawPlanarBoard(board_, size, image, image_margin_, 1, image_axis_);
			msg.encoding = image_axis_ ? sensor_msgs::image_encodings::RGB8 : sensor_msgs::image_encodings::MONO8;
		} else {
			// empty map
			image.create(size, CV_8UC1);
			image.setTo(cv::Scalar::all(255));
			msg.encoding = sensor_msgs::image_encodings::MONO8;
		}

		msg.image = image;
		img_pub_.publish(msg.toImageMsg());
	}
};

PLUGINLIB_EXPORT_CLASS(ArucoMap, nodelet::Nodelet)
