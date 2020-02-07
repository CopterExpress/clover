#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <std_msgs/String.h>

#include <clover/Execute.h>

ros::Duration timeout;

// TODO: handle timeout
std::string exec(const char *cmd) {
	ros::Time start = ros::Time::now();

	std::array<char, 128> buffer;
	std::string result;
	std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
	if (!pipe) {
		throw std::runtime_error("popen() failed!");
	}


	while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
		result += buffer.data();

		if (ros::Time::now() - start > timeout) {
			ROS_INFO("Timeout: %s", cmd);
			return result;
		}
	}

	return result;
}

bool handle(clover::Execute::Request& req, clover::Execute::Response& res)
{
	ROS_INFO("Execute: %s", req.cmd.c_str());
	res.output = exec(req.cmd.c_str());
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "shell");
	ros::NodeHandle nh, nh_priv("~");

	timeout = ros::Duration(nh_priv.param("timeout", 3.0));

	auto gt_serv = nh.advertiseService("exec", &handle);

	ROS_INFO("shell: ready");
	ros::spin();
}
