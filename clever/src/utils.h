#pragma once
// TODO: merge with util.h

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

// Read required param or shutdown the node
template<typename T>
static void param(ros::NodeHandle nh, const std::string& param_name, T& param_val)
{
	if (!nh.getParam(param_name, param_val)) {
		ROS_FATAL("Required param %s is not defined", param_name);
		ros::shutdown();
	}
}

static inline double hypot(double x, double y, double z)
{
	return std::sqrt(x*x + y*y + z*z);
}

static inline void vectorToPoint(const geometry_msgs::Vector3& vector, geometry_msgs::Point& point)
{
	point.x = vector.x;
	point.y = vector.y;
	point.z = vector.z;
}

static inline void pointToVector(const geometry_msgs::Point& point, geometry_msgs::Vector3& vector)
{
	vector.x = point.x;
	vector.y = point.y;
	vector.z = point.z;
}
