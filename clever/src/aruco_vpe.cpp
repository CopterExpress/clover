#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "util.h"

using namespace tf2_ros;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using std::string;

class ArucoVPE : public nodelet::Nodelet
{
public:
    ArucoVPE() :
        last_published_(0),
        lookup_timeout_(0.05)
    {}

private:
    ros::Time last_published_;
    ros::Duration lookup_timeout_;
    ros::Duration reset_timeout_;
    ros::Publisher vision_position_pub_;
    ros::Timer dummy_vision_timer_;
    string aruco_orientation_;
    bool reset_vpe_;

    void onInit()
    {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& nh_priv = getPrivateNodeHandle();

        nh_priv.param<string>("aruco_orientation", aruco_orientation_, "local_origin");
        bool use_mocap;
        nh_priv.param<bool>("use_mocap", use_mocap, false);
        nh_priv.param<bool>("reset_vpe", reset_vpe_, !use_mocap);
        double reset_timeout;
        nh_priv.param("reset_timeout", reset_timeout, 2.0);
        reset_timeout_ = ros::Duration(reset_timeout);

        static ros::Subscriber pose_sub = nh.subscribe("mavros/local_position/pose", 1, &ArucoVPE::handlePose, this);
        static ros::Subscriber aruco_pose_sub = nh.subscribe("aruco_pose/pose", 1, &ArucoVPE::handleArucoPose, this);

        vision_position_pub_ = nh.advertise<PoseStamped>(use_mocap ? "mavros/mocap/pose" : "mavros/vision_pose/pose", 1);

        ROS_INFO("aruco orientation frame: %s", aruco_orientation_.c_str());

        dummy_vision_timer_ = nh.createTimer(ros::Duration(0.5), &ArucoVPE::publishDummy, this);

        ROS_INFO("Aruco VPE initialized");
    }

    void publishDummy(const ros::TimerEvent&)
    {
        // This is published to init FCU's position estimator
        static PoseStamped ps;
        ps.header.stamp = ros::Time::now();
        ps.pose.orientation.w = 1;
        vision_position_pub_.publish(ps);
    }

    void  handlePose(const geometry_msgs::PoseStampedConstPtr& pose)
    {
        // local position is inited, stop posting dummy position
        ROS_INFO_ONCE("Got local position, stop publishing zeroes");
        dummy_vision_timer_.stop();
    }

    void handleArucoPose(const geometry_msgs::PoseStampedConstPtr& pose)
    {
        static TransformBroadcaster br;
        static Buffer tf_buffer;
        static TransformListener tfListener(tf_buffer);
        static StaticTransformBroadcaster static_br;
        static PoseStamped ps, vpe_raw, vpe;
        TransformStamped t;

        ros::Time stamp = pose->header.stamp;
        double roll, pitch, yaw;

        try
        {
            // Refine aruco map pose
            // Reference in local origin
            t = tf_buffer.lookupTransform(aruco_orientation_, "aruco_map_reference", stamp, lookup_timeout_);
            quaternionToEuler(t.transform.rotation, roll, pitch, yaw);
            eulerToQuaternion(t.transform.rotation, 0, 0, yaw);
            t.child_frame_id = "aruco_map_reference_horiz";
            br.sendTransform(t);

            // Aruco map in reference
            t = tf_buffer.lookupTransform("aruco_map_reference", "aruco_map_raw", stamp, lookup_timeout_);
            t.header.frame_id = "aruco_map_reference_horiz";
            t.child_frame_id = "aruco_map_vision";
            br.sendTransform(t);

            if (last_published_.toSec() == 0 || // no vpe has been posted
               (reset_vpe_ && (ros::Time::now() - last_published_ > reset_timeout_))) // vpe origin outdated
            {
                ROS_INFO("Reset VPE");
                t = tf_buffer.lookupTransform("local_origin", "aruco_map_vision", stamp, lookup_timeout_);
                t.child_frame_id = "aruco_map";
                static_br.sendTransform(t);
            }

            // Calculate VPE
            ps.header.frame_id = "fcu_horiz";
            ps.header.stamp = stamp;
            ps.pose.orientation.w = 1;

            tf_buffer.transform(ps, vpe_raw, "aruco_map_vision", lookup_timeout_);

            vpe_raw.header.frame_id = "aruco_map";
            tf_buffer.transform(vpe_raw, vpe, "local_origin", lookup_timeout_);

            vision_position_pub_.publish(vpe);

            last_published_ = stamp;
            dummy_vision_timer_.stop();
        }
        catch (const tf2::TransformException& e)
        {
            ROS_WARN_THROTTLE(10, "Aruco VPE: failed to transform: %s", e.what());
        }
    }
};

PLUGINLIB_EXPORT_CLASS(ArucoVPE, nodelet::Nodelet)
