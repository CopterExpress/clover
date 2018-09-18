/*
 * Optical Flow node for PX4
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <vector>
#include <cmath>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>

using cv::Mat;

class OpticalFlow : public nodelet::Nodelet
{
public:
	OpticalFlow():
		camera_matrix_(3, 3, CV_64F),
		dist_coeffs_(8, 1, CV_64F),
		tf_listener_(tf_buffer_)
	{}

private:
	ros::Publisher flow_pub_, velo_pub_, shift_pub_;
	ros::Time prev_stamp_;
	std::string fcu_frame_id_;
	image_transport::CameraSubscriber img_sub_;
	image_transport::Publisher img_pub_;
	mavros_msgs::OpticalFlowRad flow_;
	int roi_, roi_2_;
	Mat hann_;
	Mat prev_, curr_;
	Mat camera_matrix_, dist_coeffs_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	void onInit()
	{
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& nh_priv = getPrivateNodeHandle();
		image_transport::ImageTransport it(nh);
		image_transport::ImageTransport it_priv(nh_priv);

		nh_priv.param<std::string>("mavros/local_position/tf/child_frame_id", fcu_frame_id_, "fcu");
		nh_priv.param("roi", roi_, 128);
		roi_2_ = roi_ / 2;

		img_sub_ = it.subscribeCamera("image", 1, &OpticalFlow::flow, this);
		img_pub_ = it_priv.advertise("debug", 1);
		flow_pub_ = nh.advertise<mavros_msgs::OpticalFlowRad>("mavros/px4flow/raw/send", 1);
		velo_pub_ = nh_priv.advertise<geometry_msgs::TwistStamped>("angular_velocity", 1);
		shift_pub_ = nh_priv.advertise<geometry_msgs::Vector3Stamped>("shift", 1);

		flow_.integrated_xgyro = NAN; // no IMU available
		flow_.integrated_ygyro = NAN;
		flow_.integrated_zgyro = NAN;
		flow_.time_delta_distance_us = 0;
		flow_.distance = -1; // no distance sensor available
		flow_.temperature = 0;

		ROS_INFO("Optical Flow initialized");
	}

	void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo) {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				camera_matrix_.at<double>(i, j) = cinfo->K[3 * i + j];
			}
		}
		for (int k = 0; k < cinfo->D.size(); k++) {
			dist_coeffs_.at<double>(k) = cinfo->D[k];
		}
	}

	void drawFlow(Mat& frame, double x, double y, double quality) const
	{
		double brightness = (1 - quality) * 25;;
		cv::Scalar color(brightness, brightness, brightness);
		double radius = std::sqrt(x * x + y * y);

		// draw a circle and line indicating the shift direction...
		cv::Point center(frame.cols >> 1, frame.rows >> 1);
		cv::circle(frame, center, (int)(radius*5), color, 3, cv::LINE_AA);
		cv::line(frame, center, cv::Point(center.x + (int)(x*5), center.y + (int)(y*5)), color, 3, cv::LINE_AA);
	}

	void flow(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cinfo)
	{
		parseCameraInfo(cinfo);

		auto img = cv_bridge::toCvShare(msg, "mono8")->image;

		// Apply ROI
		if (roi_ != 0) {
			img = img(cv::Rect((msg->width / 2 - roi_2_), (msg->height / 2 - roi_2_), roi_, roi_));
		}

		img.convertTo(curr_, CV_64F);

		if (prev_.empty()) {
			prev_ = curr_.clone();
			prev_stamp_ = msg->header.stamp;
			cv::createHanningWindow(hann_, curr_.size(), CV_64F);

		} else {
			double response;
			cv::Point2d shift = cv::phaseCorrelate(prev_, curr_, hann_, &response);

			// Publish raw shift in pixels
			static geometry_msgs::Vector3Stamped shift_vec;
			shift_vec.header.stamp = msg->header.stamp;
			shift_vec.header.frame_id = msg->header.frame_id;
			shift_vec.vector.x = shift.x;
			shift_vec.vector.y = shift.y;
			shift_pub_.publish(shift_vec);

			// Undistort flow in pixels
			uint32_t flow_center_x = msg->width / 2;
			uint32_t flow_center_y = msg->height / 2;
			shift.x += flow_center_x;
			shift.y += flow_center_y;

			std::vector<cv::Point2d> points_dist = { shift };
			std::vector<cv::Point2d> points_undist(1);

			cv::undistortPoints(points_dist, points_undist, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
			points_undist[0].x -= flow_center_x;
			points_undist[0].y -= flow_center_y;

			// Calculate flow in radians
			double focal_length_x = camera_matrix_.at<double>(0, 0);
			double focal_length_y = camera_matrix_.at<double>(1, 1);
			double flow_x = atan2(points_undist[0].x, focal_length_x);
			double flow_y = atan2(points_undist[0].y, focal_length_y);

			// // Convert to FCU frame
			static geometry_msgs::Vector3Stamped flow_camera, flow_fcu;
			flow_camera.header.frame_id = msg->header.frame_id;
			flow_camera.header.stamp = msg->header.stamp;
			flow_camera.vector.x = flow_y; // +y means counter-clockwise rotation around Y axis
			flow_camera.vector.y = -flow_x; // +x means clockwise rotation around X axis
			tf_buffer_.transform(flow_camera, flow_fcu, fcu_frame_id_);

			// Calculate integration time
			ros::Duration integration_time = msg->header.stamp - prev_stamp_;
			uint32_t integration_time_us = integration_time.toSec() * 1.0e6;

			// Publish flow in fcu frame
			flow_.header.stamp = /*prev_stamp_*/ msg->header.stamp;
			flow_.integration_time_us = integration_time_us;
			flow_.integrated_x = flow_fcu.vector.x;
			flow_.integrated_y = flow_fcu.vector.y;
			flow_.quality = (uint8_t)(response * 255);
			flow_pub_.publish(flow_);

			// Publish debug image
			if (img_pub_.getNumSubscribers() > 0) {
				// publish debug image
				drawFlow(img, shift_vec.vector.x, shift_vec.vector.y, response);
				cv_bridge::CvImage out_msg;
				out_msg.header.frame_id = msg->header.frame_id;
				out_msg.header.stamp = msg->header.stamp;
				out_msg.encoding = sensor_msgs::image_encodings::MONO8;
				out_msg.image = img;
				img_pub_.publish(out_msg.toImageMsg());
			}

			// Publish estimated angular velocity
			static geometry_msgs::TwistStamped velo;
			velo.header.stamp = msg->header.stamp;
			velo.header.frame_id = fcu_frame_id_;
			velo.twist.angular.x = flow_.integrated_x / integration_time.toSec();
			velo.twist.angular.y = flow_.integrated_y / integration_time.toSec();
			velo_pub_.publish(velo);

			prev_ = curr_.clone();
			prev_stamp_ = msg->header.stamp;
		}
	}
};

PLUGINLIB_EXPORT_CLASS(OpticalFlow, nodelet::Nodelet)
