#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDStateArray.h>

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

class LedControllerPlugin : public gazebo::ModelPlugin {
private:
	std::unique_ptr<ros::NodeHandle> nh;
	std::string ns;
	ros::ServiceServer setLedsSrv;
	led_msgs::LEDStateArray ledState;
	ros::Publisher statePublisher;
	std::mutex handleMutex;

public:
	bool setLeds(led_msgs::SetLEDs::Request &req, led_msgs::SetLEDs::Response &resp)
	{
		std::lock_guard<std::mutex> lock(handleMutex);
		for(const auto& led : req.leds)
		{
			if (led.index < ledState.leds.size()) {
				ledState.leds[led.index].r = led.r;
				ledState.leds[led.index].g = led.g;
				ledState.leds[led.index].b = led.b;
			}
		}
		statePublisher.publish(ledState);
		resp.success = true;
		return true;
	}

	virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
	{
		ROS_INFO("Initialize LED Controller");

		// We need "libgazebo_ros_api.so" to be loaded
		if (!ros::isInitialized())
		{
			ROS_FATAL_NAMED("LedController", "Tried to load ROS plugin when ROS Gazebo API is not loaded. Please use gazebo_ros node to"
			                                 "launch Gazebo.");
		}

		ns = "";

		if (sdf->HasElement("robotNamespace")) {
			ns = sdf->Get<std::string>("robotNamespace");
		}
		if (!sdf->HasElement("ledCount")) {
			gzerr << "ledCount is not set, but is required for the plugin to function correctly\n";
			return;
		}
		int totalLeds = sdf->Get<int>("ledCount");
		ledState.leds.resize(totalLeds);
		for (int i = 0; i < totalLeds; i++) {
			ledState.leds[i].index = i;
		}

		nh.reset(new ros::NodeHandle(ns));

		setLedsSrv = nh->advertiseService("led/set_leds", &LedControllerPlugin::setLeds, this);
		statePublisher = nh->advertise<led_msgs::LEDStateArray>("led/state", 1, true);

		statePublisher.publish(ledState);
	}
};

GZ_REGISTER_MODEL_PLUGIN(LedControllerPlugin);
