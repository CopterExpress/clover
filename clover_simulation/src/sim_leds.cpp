#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDStateArray.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>

#if GAZEBO_MAJOR_VERSION >= 9
#include <ignition/math/Color.hh>
using GazeboColor = ignition::math::Color;
#else
#include <gazebo/common/Color.hh>
using GazeboColor = gazebo::common::Color;
#endif

#include <string>
#include <unordered_map>
#include <mutex>
#include <vector>

#include <thread>

namespace sim_led
{
// Forward declaration of the plugin for led controller
class LedVisualPlugin;
} // 

namespace led_controller
{

/// LedController: a class that provides ROS interface for the LEDs.
class LedController
{
private:
	/// Role for the current controller
	enum class Role
	{
		Client, // Client: runs on /gazebo_gui, responsible for "preview window"
		Server  // Server: runs on /gazebo, responsible for renders on Gazebo sensors
	} role;

	// Pointers to the LED plugins that we know about
	std::vector<sim_led::LedVisualPlugin*> registeredLeds;
	// Mutex protecting the vector
	std::mutex registryMutex;
	std::string robotNamespace;

	std::unique_ptr<ros::NodeHandle> nh;

	ros::ServiceServer setLedsSrv;
	// Note: LED state should only be published by the /gazebo node
	led_msgs::LEDStateArray ledState;
	ros::Publisher statePublisher;
	// LED state will be read from the topic to avoid creating more services
	ros::Subscriber stateSubscriber;

	bool setLeds(led_msgs::SetLEDs::Request& req, led_msgs::SetLEDs::Response& resp);
	void handleLedsMsg(const led_msgs::LEDStateArrayConstPtr& leds);

public:
	LedController(const std::string& robotNamespace) : robotNamespace(robotNamespace)
	{
		// We need "libgazebo_ros_api.so" to be loaded
		if (!ros::isInitialized())
		{
			ROS_FATAL_NAMED("LedController", "Tried to load ROS plugin when ROS Gazebo API is not loaded. Please use gazebo_ros node to"
			                                 "launch Gazebo.");
		}

		role = (ros::this_node::getName() == "/gazebo") ? Role::Server : Role::Client;
		ROS_INFO_NAMED(("LedController_" + robotNamespace).c_str(), "LedController has started (as %s)", role == Role::Client ? "client" : "server");

		nh.reset(new ros::NodeHandle(robotNamespace));
		if (role == Role::Server)
		{
			setLedsSrv = nh->advertiseService("led/set_leds", &LedController::setLeds, this);
			statePublisher = nh->advertise<led_msgs::LEDStateArray>("led/state", 1, true);
		}
		else
		{
			// LED state should be published to the "led/state" topic, so we grab our data there
			stateSubscriber = nh->subscribe<led_msgs::LEDStateArray>("led/state", 1, &LedController::handleLedsMsg, this);
		}
	};

	~LedController()
	{
		nh->shutdown();
	}

	void registerPlugin(sim_led::LedVisualPlugin* plugin, int ledIdx = 0, int totalLeds = 0)
	{
		assert(ledIdx < totalLeds);
		std::lock_guard<std::mutex> lock(registryMutex);
		if (totalLeds > 0) {
			registeredLeds.resize(totalLeds);
			ledState.leds.resize(totalLeds);
		}
		ROS_DEBUG_NAMED(("LedController_" + robotNamespace).c_str(), "Registering LED visual plugin to %s (LED id=%d)", (role == Role::Client) ? "client" : "server", ledIdx);
		registeredLeds[ledIdx] = plugin;
		ledState.leds[ledIdx].index = ledIdx;
		if (role == Role::Server)
			statePublisher.publish(ledState);
	}

	void unregisterPlugin(sim_led::LedVisualPlugin* plugin)
	{
		std::lock_guard<std::mutex> lock(registryMutex);
		auto it = std::find(registeredLeds.begin(), registeredLeds.end(), plugin);
		if (it != registeredLeds.end())
		{
			ROS_DEBUG_STREAM_NAMED(("LedController_" + robotNamespace).c_str(), "Unregistering LED visual plugin (LED id=" << (it - registeredLeds.begin()) << ")");
			*it = nullptr;
		}
	}
};

// Guards controllers map (static to led_controller::get())
std::mutex controllerMutex;

LedController& get(std::string robotNamespace)
{
	static std::unordered_map<std::string, std::unique_ptr<LedController>> controllers;
	std::lock_guard<std::mutex> lock(controllerMutex);
	auto it = controllers.find(robotNamespace);
	if (it == controllers.end()) {
		gzwarn << "Creating new LED controller for namespace " << robotNamespace << "\n";
		controllers[robotNamespace].reset(new LedController(robotNamespace));
		return *controllers[robotNamespace];
	}
	return *(it->second);
}

} // led_controller


namespace sim_led
{

class LedVisualPlugin : public gazebo::VisualPlugin {
private:
	std::string ns;
	gazebo::rendering::VisualPtr vptr;

public:
	void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override {
		// FIXME: This is a fragile way to guess our index
		// FIXME: This is based on an assumption that the parent will have a mangled name
		// FIXME: (like led_strip::base_link::base_link_fixed_joint_lump__led_00_link_visual_1)
		// FIXME: This will obviously break if gazebo decides to go with something else
		auto parentName = sdf->GetParent()->GetAttribute("name")->GetAsString();
		auto lastDashPos = parentName.rfind('_');
		int myIndex = 0;
		if (lastDashPos != std::string::npos)
		{
			auto indexStr = parentName.substr(lastDashPos + 1);
			try {
				myIndex = std::stoi(indexStr);
			} catch(const std::exception &e) {
				gzwarn << "Failed to convert " << indexStr << " to integer: " << e.what() <<  ", assuming 0\n";
				myIndex = 0;
			}
		}
		else
		{
			gzerr << "Failed to parse parent name: " << parentName << "; could not determine our index\n";
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

		vptr = visual;
		led_controller::get(ns).registerPlugin(this, myIndex, totalLeds);
	}

	~LedVisualPlugin()
	{
		led_controller::get(ns).unregisterPlugin(this);
	}

	void SetColor(const GazeboColor& emissive)
	{
		vptr->SetEmissive(emissive);
	}

};
}

// FIXME: These two functions basically do the same thing, maybe they can be merged?
bool led_controller::LedController::setLeds(led_msgs::SetLEDs::Request &req, led_msgs::SetLEDs::Response &resp)
{
	std::lock_guard<std::mutex> lock(registryMutex);
	for(const auto& led : req.leds)
	{
		if (led.index < registeredLeds.size()) {
			auto color = GazeboColor(led.r / 255.0f, led.g / 255.0f, led.b / 255.0f);
			auto ledPlugin = registeredLeds[led.index];
			if (ledPlugin) ledPlugin->SetColor(color);
			ledState.leds[led.index].r = led.r;
			ledState.leds[led.index].g = led.g;
			ledState.leds[led.index].b = led.b;
		}
	}
	statePublisher.publish(ledState);
	resp.success = true;
	return true;
}

void led_controller::LedController::handleLedsMsg(const led_msgs::LEDStateArrayConstPtr& leds)
{
	std::lock_guard<std::mutex> lock(registryMutex);
	for(const auto& led : leds->leds)
	{
		if (led.index < registeredLeds.size())
		{
			auto color = GazeboColor(led.r / 255.0f, led.g / 255.0f, led.b / 255.0f);
			auto ledPlugin = registeredLeds[led.index];
			if (ledPlugin) ledPlugin->SetColor(color);
		}
	}
}

GZ_REGISTER_VISUAL_PLUGIN(sim_led::LedVisualPlugin);
