/*
 * Universal VPE publisher node
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <aruco_pose/MarkerArray.h>

#include "utils.h"

using std::string;

static string frame_id, child_frame_id;
static tf2_ros::Buffer tf_buffer;
static ros::Publisher vpe_pub;
static ros::Subscriber local_position_sub;
static ros::Timer zero_timer;
static geometry_msgs::PoseStamped vpe, pose;
static ros::Time local_pose_stamp(0);
static ros::Duration publish_zero_timout;
tf2_ros::TransformBroadcaster br;

void publishZero(const ros::TimerEvent&)
{
	if ((ros::Time::now() - pose.header.stamp < publish_zero_timout) ||
	    (ros::Time::now() - vpe.header.stamp < publish_zero_timout))
		return;

	ROS_INFO_THROTTLE(10, "vpe_publisher: publish zero");
	vpe.header.stamp = ros::Time::now();
	vpe.pose.orientation.w = 1;
	vpe_pub.publish(vpe);
}

void localPositionCallback(const geometry_msgs::PoseStamped& msg)
{
	pose = msg;
}

template <typename T>
void callback(const T& msg)
{
	try {
		auto transform = tf_buffer.lookupTransform(frame_id, child_frame_id,
		                                            msg->header.stamp, ros::Duration(0.02));
		                                            vpe.header.stamp = msg->header.stamp;
		vpe.pose.position.x = transform.transform.translation.x;
		vpe.pose.position.y = transform.transform.translation.y;
		vpe.pose.position.z = transform.transform.translation.z;
		vpe.pose.orientation = transform.transform.rotation;
		vpe.header.stamp = msg->header.stamp;
		vpe_pub.publish(vpe);
	} catch (const tf2::TransformException& e) {
		ROS_WARN_THROTTLE(10, "vpe_publisher: %s", e.what());
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "vpe_publisher");
	ros::NodeHandle nh, nh_priv("~");

	tf2_ros::TransformListener tf_listener(tf_buffer);

	param(nh_priv, "frame_id", frame_id);
	param(nh_priv, "child_frame_id", child_frame_id);
	nh_priv.param<std::string>("mavros/local_position/frame_id", vpe.header.frame_id, "map");

	auto pose_sub = nh_priv.subscribe<geometry_msgs::PoseStamped>("pose", 1, &callback);
	auto pose_cov_sub = nh_priv.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 1, &callback);
	auto markers_sub = nh_priv.subscribe<aruco_pose::MarkerArray>("markers", 1, &callback);
	//auto pose_cov_sub = nh_priv.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 1, &callback);

	vpe_pub = nh_priv.advertise<geometry_msgs::PoseStamped>("pose_pub", 1);
	//vpe_cov_pub = nh_priv_.advertise<geometry_msgs::PoseStamped>("pose_cov_pub", 1);

	if (nh_priv.param("publish_zero", false)) {
		// publish zero to initialize the local position
		vpe.header.stamp = ros::Time(0);
		zero_timer = nh.createTimer(ros::Duration(0.1), &publishZero);
		publish_zero_timout = ros::Duration(nh_priv.param("publish_zero_timout", 5.0));
		local_position_sub = nh.subscribe("mavros/local_position/pose", 1, &localPositionCallback);
	}

	ROS_INFO("vpe_publisher: ready");
	ros::spin();
}
