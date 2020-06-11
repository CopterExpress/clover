#include <ros/ros.h>
#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDStateArray.h>
#include <ros/console.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <iostream>
#include <string>

static gazebo::transport::PublisherPtr pub;

led_msgs::LEDStateArray strip_state;
ros::Publisher led_state_pub;

void publishLedState()
{
	led_state_pub.publish(strip_state);
}

bool setLeds(led_msgs::SetLEDs::Request& req, led_msgs::SetLEDs::Response& resp)
{
	//TODO: check out of bounds

	for (auto const& led : req.leds) {
		gazebo::msgs::Visual msg;
		msg.set_type(gazebo::msgs::Visual::VISUAL);

		std::string num = std::to_string(led.index);

		msg.set_name("led_strip::led_" + num + "_link::led_" + num + "_visual");
		msg.set_parent_name("led_strip::led_" + num + "_link");

		msg.mutable_material()->mutable_emissive()->set_r(led.r / 255.0);
		msg.mutable_material()->mutable_emissive()->set_g(led.g / 255.0);
		msg.mutable_material()->mutable_emissive()->set_b(led.b / 255.0);

		pub->Publish(msg);

		strip_state.leds[led.index].r = led.r;
		strip_state.leds[led.index].g = led.g;
		strip_state.leds[led.index].b = led.b;
	}

	publishLedState();

	resp.success = true;
	return true;
}

int main(int argc, char **argv)
{
	// Init Gazebo node
	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	pub = node->Advertise<gazebo::msgs::Visual>("~/visual");
	pub->WaitForConnection();

	// Init ROS node
	ros::init(argc, argv, "led_strip");
	ros::NodeHandle nh, nh_priv("~");

	strip_state.leds.resize(nh_priv.param("led_count", 40));

	auto srv_leds = nh_priv.advertiseService("set_leds", setLeds);

	// Init strip state
	for(size_t i = 0; i < strip_state.leds.size(); i++) {
		strip_state.leds[i].index = i;
		strip_state.leds[i].r = 0;
		strip_state.leds[i].g = 0;
		strip_state.leds[i].b = 0;
	}

	led_state_pub = nh_priv.advertise<led_msgs::LEDStateArray>("state", 1, true);
	publishLedState();

	gazebo::msgs::Visual msg;
	msg.set_type(gazebo::msgs::Visual::VISUAL);

	ROS_INFO("gazebo led_strip: ready");
	ros::spin();

	gazebo::client::shutdown();
}
