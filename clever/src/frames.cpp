/*
 * Auxiliary TF frames for CLEVER drone kit:
 * - Body frame (drone body with zero pitch and roll).
 * - TODO: REP-0105 `odom` frame emulation: continuous frame without discrete jumps.
 * - TODO: Terrain frame (base on ALTITUDE message).
 * - TODO: map_upside_down frame
 * - TODO: home frame?
 *
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

// TODO: consider implementing as a mavros plugin

#include <string>
#include <memory>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

using std::string;

static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
static geometry_msgs::TransformStamped body;

inline void publishBody(const geometry_msgs::PoseStamped& pose)
{
	// Get only yaw from pose
	tf::Quaternion q;
	q.setRPY(0, 0, tf::getYaw(pose.pose.orientation));
	tf::quaternionTFToMsg(q, body.transform.rotation);

	body.transform.translation.x = pose.pose.position.x;
	body.transform.translation.y = pose.pose.position.y;
	body.transform.translation.z = pose.pose.position.z;
	body.header.frame_id = pose.header.frame_id;
	body.header.stamp = pose.header.stamp;
	br->sendTransform(body);
}

void poseCallback(const geometry_msgs::PoseStamped& pose)
{
	publishBody(pose);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "frames");
	ros::NodeHandle nh, nh_priv("~");

	nh_priv.param<string>("body/frame_id", body.child_frame_id, "body");

	br = std::make_shared<tf2_ros::TransformBroadcaster>();
	ros::Subscriber pose_sub = nh.subscribe("mavros/local_position/pose", 1, &poseCallback);
	ROS_INFO("frames: ready");
	ros::spin();
}
