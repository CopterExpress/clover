#pragma once

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

inline void quaternionToEuler(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
    tf::Quaternion tfq(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(tfq);
    m.getRPY(roll, pitch, yaw);
}

inline void eulerToQuaternion(geometry_msgs::Quaternion& q, double roll, double pitch, double yaw)
{
    tf::Quaternion tfq(roll, pitch, yaw);
    quaternionTFToMsg(tfq, q);
}
