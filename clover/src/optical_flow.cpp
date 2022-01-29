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
#include <tf2/utils.h>
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
		camera_matrix_(3, 3, CV_64F)
	{}

private:
	ros::Publisher flow_pub_, velo_pub_, shift_pub_;
	ros::Time prev_stamp_;
	std::string fcu_frame_id_, local_frame_id_;
	image_transport::CameraSubscriber img_sub_;
	image_transport::Publisher img_pub_;
	mavros_msgs::OpticalFlowRad flow_;
	int roi_px_;
	double roi_rad_;
	cv::Rect roi_;
	Mat hann_;
	Mat prev_, curr_;
	Mat camera_matrix_, dist_coeffs_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
	bool calc_flow_gyro_;
	float flow_gyro_default_;

	void onInit()
	{
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& nh_priv = getPrivateNodeHandle();
		image_transport::ImageTransport it(nh);
		image_transport::ImageTransport it_priv(nh_priv);

		tf_buffer_.reset(new tf2_ros::Buffer());
		tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_, nh));

		local_frame_id_ = nh.param<std::string>("mavros/local_position/tf/frame_id", "map");
		fcu_frame_id_ = nh.param<std::string>("mavros/local_position/tf/child_frame_id", "base_link");
		roi_px_ = nh_priv.param("roi", 128);
		roi_rad_ = nh_priv.param("roi_rad", 0.0);
		calc_flow_gyro_ = nh_priv.param("calc_flow_gyro", false);
		flow_gyro_default_ = nh_priv.param("flow_gyro_default", NAN);

		img_pub_ = it_priv.advertise("debug", 1);
		flow_pub_ = nh.advertise<mavros_msgs::OpticalFlowRad>("mavros/px4flow/raw/send", 1);
		velo_pub_ = nh_priv.advertise<geometry_msgs::TwistStamped>("angular_velocity", 1);
		shift_pub_ = nh_priv.advertise<geometry_msgs::Vector3Stamped>("shift", 1);

		flow_.time_delta_distance_us = 0;
		flow_.distance = -1; // no distance sensor available
		flow_.temperature = 0;

		img_sub_ = it.subscribeCamera("image_raw", 1, &OpticalFlow::flow, this);

		NODELET_INFO("Optical Flow initialized");
	}

	void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo) {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				camera_matrix_.at<double>(i, j) = cinfo->K[3 * i + j];
			}
		}
		dist_coeffs_ = cv::Mat(cinfo->D, true);
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

		if (roi_.width == 0) { // ROI is not calculated
			// Calculate ROI
			if (roi_rad_ != 0) {
				std::vector<cv::Point3f> object_points = {
					cv::Point3f(-sin(roi_rad_ / 2), -sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
					cv::Point3f(sin(roi_rad_ / 2), sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
				};

				std::vector<double> vec { 0, 0, 0 };
				std::vector<cv::Point2f> img_points;
				cv::projectPoints(object_points, vec, vec, camera_matrix_, dist_coeffs_, img_points);

				roi_ = cv::Rect(cv::Point2i(round(img_points[0].x), round(img_points[0].y)), 
					cv::Point2i(round(img_points[1].x), round(img_points[1].y)));

				ROS_INFO("ROI: %d %d - %d %d ", roi_.tl().x, roi_.tl().y, roi_.br().x, roi_.br().y);

			} else if (roi_px_ != 0) {
				roi_ = cv::Rect((msg->width / 2 - roi_px_ / 2), (msg->height / 2 - roi_px_ / 2), roi_px_, roi_px_);
			}
		}

		if (roi_.width != 0) { // ROI is set
			// Apply ROI
			img = img(roi_);
		}

		img.convertTo(curr_, CV_32F);

		if (prev_.empty()) {
			prev_ = curr_.clone();
			prev_stamp_ = msg->header.stamp;
			cv::createHanningWindow(hann_, curr_.size(), CV_32F);

		} else {
			double response;
			cv::Point2d shift = cv::phaseCorrelate(prev_, curr_, hann_, &response);

			// Publish raw shift in pixels
			geometry_msgs::Vector3Stamped shift_vec;
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

			// Convert to FCU frame
			geometry_msgs::Vector3Stamped flow_camera, flow_fcu;
			flow_camera.header.frame_id = msg->header.frame_id;
			flow_camera.header.stamp = msg->header.stamp;
			flow_camera.vector.x = flow_y; // +y means counter-clockwise rotation around Y axis
			flow_camera.vector.y = -flow_x; // +x means clockwise rotation around X axis
			try {
				tf_buffer_->transform(flow_camera, flow_fcu, fcu_frame_id_);
			} catch (const tf2::TransformException& e) {
				// transform is not available yet
				return;
			}

			// Calculate integration time
			ros::Duration integration_time = msg->header.stamp - prev_stamp_;
			uint32_t integration_time_us = integration_time.toSec() * 1.0e6;

			// Calculate flow gyro
			flow_.integrated_xgyro = flow_gyro_default_;
			flow_.integrated_ygyro = flow_gyro_default_;
			flow_.integrated_zgyro = flow_gyro_default_;

			if (calc_flow_gyro_) {
				try {
					auto flow_gyro_camera = calcFlowGyro(msg->header.frame_id, prev_stamp_, msg->header.stamp);
					geometry_msgs::Vector3Stamped flow_gyro_fcu;
					tf_buffer_->transform(flow_gyro_camera, flow_gyro_fcu, fcu_frame_id_);
					flow_.integrated_xgyro = flow_gyro_fcu.vector.x;
					flow_.integrated_ygyro = flow_gyro_fcu.vector.y;
					flow_.integrated_zgyro = flow_gyro_fcu.vector.z;
				} catch (const tf2::TransformException& e) {
					// Transform not available, keep NANs in flow gyro
				}
			}

			// Publish flow in fcu frame
			flow_.header.stamp = /*prev_stamp_*/ msg->header.stamp;
			flow_.integration_time_us = integration_time_us;
			flow_.integrated_x = flow_fcu.vector.x;
			flow_.integrated_y = flow_fcu.vector.y;
			flow_.quality = (uint8_t)(response * 255);
			flow_pub_.publish(flow_);

			prev_ = curr_.clone();
			prev_stamp_ = msg->header.stamp;

publish_debug:
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
			geometry_msgs::TwistStamped velo;
			velo.header.stamp = msg->header.stamp;
			velo.header.frame_id = fcu_frame_id_;
			velo.twist.angular.x = flow_fcu.vector.x / integration_time.toSec();
			velo.twist.angular.y = flow_fcu.vector.y / integration_time.toSec();
			velo_pub_.publish(velo);
		}
	}

	geometry_msgs::Vector3Stamped calcFlowGyro(const std::string& frame_id, const ros::Time& prev, const ros::Time& curr)
	{
		tf2::Quaternion prev_rot, curr_rot;
		tf2::fromMsg(tf_buffer_->lookupTransform(frame_id, local_frame_id_, prev).transform.rotation, prev_rot);
		tf2::fromMsg(tf_buffer_->lookupTransform(frame_id, local_frame_id_, curr, ros::Duration(0.1)).transform.rotation, curr_rot);

		geometry_msgs::Vector3Stamped flow;
		flow.header.frame_id = frame_id;
		flow.header.stamp = curr;
		// https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternion_↔_angular_velocities
		auto diff = ((curr_rot - prev_rot) * prev_rot.inverse()) * 2.0f;
		flow.vector.x = -diff.x();
		flow.vector.y = -diff.y();
		flow.vector.z = -diff.z();

		return flow;
	}
};

PLUGINLIB_EXPORT_CLASS(OpticalFlow, nodelet::Nodelet)
