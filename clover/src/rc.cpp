/*
 * Clover mobile remote control backend
 * Send ManualControl messages through UDP
 * 'latched_state' topic
 *
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ManualControl.h"
#include "mavros_msgs/Mavlink.h"

struct ControlMessage
{
	int16_t x, y, z, r;
} __attribute__((packed));

class RC
{
public:
	RC():
		nh(),
		nh_priv("~")
	{
		bool use_fake_gcs = nh_priv.param("use_fake_gcs", true);
		// Create socket thread
		std::thread t(&RC::socketThread, this);
		t.detach();

		if (use_fake_gcs) {
			std::thread gcst(&RC::fakeGCSThread, this);
			gcst.detach();
		}

		initLatchedState();
	}

private:
	ros::NodeHandle nh, nh_priv;
	ros::Subscriber state_sub;
	ros::Publisher state_pub;
	ros::Timer state_timeout_timer;
	ros::Time last_manual_control{0};
	mavros_msgs::StateConstPtr state_msg;

	void handleState(const mavros_msgs::StateConstPtr& state)
	{
		state_timeout_timer.setPeriod(ros::Duration(3), true);
		state_timeout_timer.start();

		if (!state_msg ||
			state->connected != state_msg->connected ||
			state->mode != state_msg->mode ||
			state->armed != state_msg->armed) {
				state_msg = state;
				state_pub.publish(state_msg);
			}
	}

	void stateTimedOut(const ros::TimerEvent&)
	{
		ROS_INFO("State timeout");
		mavros_msgs::State unknown_state;
		state_pub.publish(unknown_state);
		state_msg = nullptr;
	}

	void initLatchedState()
	{
		state_sub = nh.subscribe("mavros/state", 1, &RC::handleState, this);
		state_pub = nh.advertise<mavros_msgs::State>("state_latched", 1, true);
		state_timeout_timer = nh.createTimer(ros::Duration(0), &RC::stateTimedOut, this, true, false);

		// Publish initial state
		mavros_msgs::State unknown_state;
		state_pub.publish(unknown_state);
	}

	void fakeGCSThread()
	{
		// Awful workaround for fixing PX4 not sending STATUSTEXTs
		// if there is no GCS heartbeats.
		// TODO: use timer
		// TODO: remove, when PX4 get this fixed.
		ros::Publisher mavlink_pub = nh.advertise<mavros_msgs::Mavlink>("mavlink/to", 1);

		// HEARTBEAT from GCS message
		mavros_msgs::Mavlink hb;
		hb.framing_status = mavros_msgs::Mavlink::FRAMING_OK;
		hb.magic = mavros_msgs::Mavlink::MAVLINK_V20;
		hb.len = 9;
		hb.incompat_flags = 0;
		hb.compat_flags = 0;
		hb.seq = 0;
		hb.sysid = 255;
		hb.compid = 0;
		hb.checksum = 26460;
		hb.payload64.push_back(342282393542983680);
		hb.payload64.push_back(3);

		ros::Rate rate(1);
		while (ros::ok()) {
			if (ros::Time::now() - last_manual_control < ros::Duration(8)) {
				mavlink_pub.publish(hb);
			}
			rate.sleep();
		}
	}

	int createSocket(int port)
	{
		int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

		sockaddr_in sin;
		sin.sin_family = AF_INET;
		sin.sin_addr.s_addr = htonl(INADDR_ANY);
		sin.sin_port = htons(port);

		if (bind(sockfd, (sockaddr *)&sin, sizeof(sin)) < 0) {
			ROS_FATAL("socket bind error: %s", strerror(errno));
			close(sockfd);
			ros::shutdown();
		}

		return sockfd;
	}

	void socketThread()
	{
		int port;
		nh_priv.param("port", port, 35602);
		int sockfd = createSocket(port);

		char buff[9999];

		ros::Publisher manual_control_pub = nh.advertise<mavros_msgs::ManualControl>("mavros/manual_control/send", 1);
		mavros_msgs::ManualControl manual_control_msg;

		sockaddr_in client_addr;
		socklen_t client_addr_size = sizeof(client_addr);

		ROS_INFO("UDP RC initialized on port %d", port);

		while (true) {
			// read next UDP packet
			int bsize = recvfrom(sockfd, &buff[0], sizeof(buff) - 1, 0, (sockaddr *) &client_addr, &client_addr_size);

			if (bsize < 0) {
				ROS_ERROR("recvfrom() error: %s", strerror(errno));
			} else if (bsize != sizeof(ControlMessage)) {
				ROS_ERROR_THROTTLE(30, "Wrong UDP packet size: %d", bsize);
			}

			// unpack message
			// warning: ignore endianness, so the code is platform-dependent
			ControlMessage *msg = (ControlMessage *)buff;

			manual_control_msg.x = msg->x;
			manual_control_msg.y = msg->y;
			manual_control_msg.z = msg->z;
			manual_control_msg.r = msg->r;
			manual_control_pub.publish(manual_control_msg);

			last_manual_control = ros::Time::now();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rc");
	RC rc;
	ros::spin();
}
