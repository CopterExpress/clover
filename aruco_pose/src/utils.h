/*
 * Utility functions
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#pragma once

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

// Read required param or shutdown the node
template<typename T>
static void param(ros::NodeHandle nh, const std::string& param_name, T& param_val)
{
	if (!nh.getParam(param_name, param_val)) {
		ROS_FATAL("Required param %s is not defined", param_name.c_str());
		ros::shutdown();
	}
}

static void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr& cinfo, cv::Mat& matrix, cv::Mat& dist)
{
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = 0; j < 3; ++j)
			matrix.at<double>(i, j) = cinfo->K[3 * i + j];

	for (unsigned int k = 0; k < cinfo->D.size(); k++)
		dist.at<double>(k) = cinfo->D[k];
}

inline void rotatePoint(cv::Point3f& p, cv::Point3f origin, float angle)
{
	float s = sin(angle);
	float c = cos(angle);

	// translate point back to origin:
	p.x -= origin.x;
	p.y -= origin.y;

	// rotate point
	float xnew = p.x * c - p.y * s;
	float ynew = p.x * s + p.y * c;

	// translate point back:
	p.x = xnew + origin.x;
	p.y = ynew + origin.y;
}

inline void fillPose(geometry_msgs::Pose& pose, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
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

inline void fillTransform(geometry_msgs::Transform& transform, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
	transform.translation.x = tvec[0];
	transform.translation.y = tvec[1];
	transform.translation.z = tvec[2];

	double angle = norm(rvec);
	cv::Vec3d axis = rvec / angle;

	tf2::Quaternion q;
	q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

	transform.rotation.w = q.w();
	transform.rotation.x = q.x();
	transform.rotation.y = q.y();
	transform.rotation.z = q.z();
}

inline void fillTranslation(geometry_msgs::Vector3& translation, const cv::Vec3d& tvec)
{
	translation.x = tvec[0];
	translation.y = tvec[1];
	translation.z = tvec[2];
}

inline bool isFlipped(tf::Quaternion& q)
{
	double yaw, pitch, roll;
	tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
	return (abs(pitch) > M_PI / 2) || (abs(roll) > M_PI / 2);
}

/* Set roll and pitch from "from" to "to", keeping yaw */
inline void snapOrientation(geometry_msgs::Quaternion& to, const geometry_msgs::Quaternion& from, bool auto_flip = false)
{
	tf::Quaternion _from, _to;
	tf::quaternionMsgToTF(from, _from);
	tf::quaternionMsgToTF(to, _to);

	if (auto_flip) {
		if (!isFlipped(_from)) {
			static const tf::Quaternion flip = tf::createQuaternionFromRPY(M_PI, 0, 0);
			_from *= flip; // flip "from"
		}
	}

	auto diff = tf::Matrix3x3(_to).transposeTimes(tf::Matrix3x3(_from));
	double _, yaw;
	diff.getRPY(_, _, yaw);
	auto q = tf::createQuaternionFromRPY(0, 0, -yaw);
	_from = _from * q; // set yaw from "to" to "from"
	tf::quaternionTFToMsg(_from, to); // set "from" to "to"
}

inline void transformToPose(const geometry_msgs::Transform& transform, geometry_msgs::Pose& pose)
{
	pose.position.x = transform.translation.x;
	pose.position.y = transform.translation.y;
	pose.position.z = transform.translation.z;
	pose.orientation = transform.rotation;
}
