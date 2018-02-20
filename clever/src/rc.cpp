// CLEVER mobile remote control support:
// * Send ManualControl messages through UDP
// * `latched_state` topic

#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ManualControl.h"

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
        // Create socket thread
        std::thread t(&RC::socketThread, this);
        t.detach();

        initLatchedState();
    }

private:
    ros::NodeHandle nh, nh_priv;
    ros::Subscriber state_sub;
    ros::Publisher state_pub;
    ros::Timer state_timeout_timer;
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
        unknown_state.connected = true;
        unknown_state.mode = "UNKNOWN";
        state_pub.publish(unknown_state);
        state_msg = nullptr;
    }

    void initLatchedState()
    {
        state_sub = nh.subscribe("mavros/state", 1, &RC::handleState, this);
        state_pub = nh.advertise<mavros_msgs::State>("state_latched", 1, true);
        state_timeout_timer = nh.createTimer(ros::Duration(0), &RC::stateTimedOut, this, true, false);
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
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc");
    RC rc;
    ros::spin();
}
