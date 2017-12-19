#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "util.h"

class FcuHoriz : public nodelet::Nodelet
{
    geometry_msgs::TransformStamped t_;

    void handlePose(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        static tf2_ros::TransformBroadcaster br;
        double roll, pitch, yaw;

        t_.header.stamp = msg->header.stamp;
        t_.header.frame_id = msg->header.frame_id;
        t_.transform.translation.x = msg->pose.position.x;
        t_.transform.translation.y = msg->pose.position.y;
        t_.transform.translation.z = msg->pose.position.z;

        // Warning: this is not thead-safe
        quaternionToEuler(msg->pose.orientation, roll, pitch, yaw);
        eulerToQuaternion(t_.transform.rotation, 0, 0, yaw);

        br.sendTransform(t_);
    }

    void onInit()
    {
        t_.child_frame_id = "fcu_horiz";
        t_.transform.rotation.w = 1;
        static ros::Subscriber pose_sub = getNodeHandle().subscribe("mavros/local_position/pose", 1, &FcuHoriz::handlePose, this);
        ROS_INFO("fcu_horiz initialized");
    }
};

PLUGINLIB_EXPORT_CLASS(FcuHoriz, nodelet::Nodelet)
