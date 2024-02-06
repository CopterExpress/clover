/*
 * High level control for the LED strip
 * Indicate flight events with the LED strip
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <clover/SetLEDEffect.h>
#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDState.h>
#include <led_msgs/LEDStateArray.h>

#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <rosgraph_msgs/Log.h>

clover::SetLEDEffect::Request current_effect;
int led_count;
ros::Timer timer;
ros::Time start_time;
double blink_rate, blink_fast_rate, flash_delay, fade_period, wipe_period, rainbow_period;
double low_battery_threshold;
std::vector<std::string> error_ignore;
led_msgs::SetLEDs set_leds;
led_msgs::LEDStateArray state, start_state;
ros::ServiceClient set_leds_srv;
mavros_msgs::State mavros_state;
int counter;

void callSetLeds()
{
	bool res = set_leds_srv.call(set_leds);
	if (!res) {
		ROS_WARN_THROTTLE(5, "Error calling set_leds service");
	} else if (!set_leds.response.success) {
		ROS_WARN_THROTTLE(5, "Calling set_leds failed: %s", set_leds.response.message.c_str());
	}
}

void rainbow(uint8_t n, uint8_t& r, uint8_t& g, uint8_t& b)
{
	if (n < 255 / 3) {
		r = n * 3;
		g = 255 - n * 3;
		b = 0;
	} else if (n < 255 / 3 * 2) {
		n -= 255 / 3;
		r = 255 - n * 3;
		g = 0;
		b = n * 3;
	} else {
		n -= 255 / 3 * 2;
		r = 0;
		g = n * 3;
		b = 255 - n * 3;
	}
}

void fill(uint8_t r, uint8_t g, uint8_t b)
{
	set_leds.request.leds.resize(led_count);
	for (int i = 0; i < led_count; i++) {
		set_leds.request.leds[i].index = i;
		set_leds.request.leds[i].r = r;
		set_leds.request.leds[i].g = g;
		set_leds.request.leds[i].b = b;
	}
	callSetLeds();
}

void proceed(const ros::TimerEvent& event)
{
	counter++;
	uint8_t r, g, b;
	set_leds.request.leds.clear();
	set_leds.request.leds.resize(led_count);

	if (current_effect.effect == "blink" || current_effect.effect == "blink_fast") {
		// enable on odd counter
		if (counter % 2 != 0) {
			fill(current_effect.r, current_effect.g, current_effect.b);
		} else {
			fill(0, 0, 0);
		}

	} else if (current_effect.effect == "fade") {
		// fade all leds from starting state
		double passed = std::min((event.current_real - start_time).toSec() / fade_period, 1.0);
		double one_minus_passed = 1 - passed;
		for (int i = 0; i < led_count; i++) {
			set_leds.request.leds[i].index = i;
			set_leds.request.leds[i].r = one_minus_passed * start_state.leds[i].r + passed * current_effect.r;
			set_leds.request.leds[i].g = one_minus_passed * start_state.leds[i].g + passed * current_effect.g;
			set_leds.request.leds[i].b = one_minus_passed * start_state.leds[i].b + passed * current_effect.b;
		}
		callSetLeds();
		if (passed >= 1.0) {
			// fade finished
			timer.stop();
		}

	} else if (current_effect.effect == "wipe") {
		set_leds.request.leds.resize(1);
		set_leds.request.leds[0].index = counter - 1;
		set_leds.request.leds[0].r = current_effect.r;
		set_leds.request.leds[0].g = current_effect.g;
		set_leds.request.leds[0].b = current_effect.b;
		callSetLeds();
		if (counter == led_count) {
			// wipe finished
			timer.stop();
		}

	} else if (current_effect.effect == "rainbow_fill") {
		rainbow(counter % 255, r, g, b);
		for (int i = 0; i < led_count; i++) {
			set_leds.request.leds[i].index = i;
			set_leds.request.leds[i].r = r;
			set_leds.request.leds[i].g = g;
			set_leds.request.leds[i].b = b;
		}
		callSetLeds();

	} else if (current_effect.effect == "rainbow") {
		for (int i = 0; i < led_count; i++) {
			int pos = (int)round(counter + (255.0 * i / led_count)) % 255;
			rainbow(pos % 255, r, g, b);
			set_leds.request.leds[i].index = i;
			set_leds.request.leds[i].r = r;
			set_leds.request.leds[i].g = g;
			set_leds.request.leds[i].b = b;
		}
		callSetLeds();
	}
}

bool setEffect(clover::SetLEDEffect::Request& req, clover::SetLEDEffect::Response& res)
{
	res.success = true;

	if (req.effect == "") {
		req.effect = "fill";
	}

	if (req.effect != "flash" && req.effect != "fill" && current_effect.effect == req.effect &&
	    current_effect.r == req.r && current_effect.g == req.g && current_effect.b == req.b) {
		res.message = "Effect already set, skip";
		return true;
	}

	if (req.effect == "fill") {
		fill(req.r, req.g, req.b);

	} else if (req.effect == "blink") {
		timer.setPeriod(ros::Duration(1 / blink_rate), true);
		timer.start();

	} else if (req.effect == "blink_fast") {
		timer.setPeriod(ros::Duration(1 / blink_fast_rate), true);
		timer.start();

	} else if (req.effect == "fade") {
		timer.setPeriod(ros::Duration(0.05), true);
		timer.start();

	} else if (req.effect == "wipe") {
		timer.setPeriod(ros::Duration(wipe_period / led_count), true);
		timer.start();

	} else if (req.effect == "flash") {
		ros::Duration delay(flash_delay);
		fill(0, 0, 0);
		delay.sleep();
		fill(req.r, req.g, req.b);
		delay.sleep();
		fill(0, 0, 0);
		delay.sleep();
		fill(req.r, req.g, req.b);
		delay.sleep();
		fill(0, 0, 0);
		delay.sleep();
		if (current_effect.effect == "fill"||
		    current_effect.effect == "fade" ||
		    current_effect.effect == "wipe") {
			// restore previous filling
			for (int i = 0; i < led_count; i++) {
				fill(current_effect.r, current_effect.g, current_effect.b);
			}
			callSetLeds();
		}
		return true; // this effect happens only once

	} else if (req.effect == "rainbow_fill") {
		timer.setPeriod(ros::Duration(rainbow_period / 255), true);
		timer.start();

	} else if (req.effect == "rainbow") {
		timer.setPeriod(ros::Duration(rainbow_period / 255), true);
		timer.start();

	} else {
		res.message = "Unknown effect: " + req.effect + ". Available effects are fill, fade, wipe, blink, blink_fast, flash, rainbow, rainbow_fill.";
		ROS_ERROR("%s", res.message.c_str());
		res.success = false;
		return true;
	}

	// set current effect
	current_effect = req;
	counter = 0;
	start_state = state;
	start_time = ros::Time::now();
	proceed({ .current_real = start_time });

	return true;
}

void handleState(const led_msgs::LEDStateArray& msg)
{
	state = msg;
	led_count = state.leds.size();
}

void notify(const std::string& event)
{
	if (ros::param::has("~notify/" + event + "/effect") ||
	    ros::param::has("~notify/" + event + "/r") ||
	    ros::param::has("~notify/" + event + "/g") ||
	    ros::param::has("~notify/" + event + "/b")) {
		ROS_INFO_THROTTLE(5, "led: notify %s", event.c_str());
		clover::SetLEDEffect effect;
		effect.request.effect = ros::param::param("~notify/" + event + "/effect", std::string(""));
		effect.request.r = ros::param::param("~notify/" + event + "/r", 0);
		effect.request.g = ros::param::param("~notify/" + event + "/g", 0);
		effect.request.b = ros::param::param("~notify/" + event + "/b", 0);
		setEffect(effect.request, effect.response);
	}
}

void handleMavrosState(const mavros_msgs::State& msg)
{
	if (msg.connected && !mavros_state.connected) {
		notify("connected");
	} else if (!msg.connected && mavros_state.connected) {
		notify("disconnected");
	} else if (msg.armed && !mavros_state.armed) {
		notify("armed");
	} else if (!msg.armed && mavros_state.armed) {
		notify("disarmed");
	} else if (msg.mode != mavros_state.mode) {
		// mode changed
		std::string mode = boost::algorithm::to_lower_copy(msg.mode);
		if (mode.find(".") != std::string::npos) {
			// remove the part before "."
			mode = mode.substr(mode.find(".") + 1);
		}
		std::string err;
		if (ros::names::validate(mode, err)) {
			notify(mode);
		}
	}
	mavros_state = msg;
}

void handleLog(const rosgraph_msgs::Log& log)
{
	if (log.level >= rosgraph_msgs::Log::ERROR) {
		// check if ignored
		for (auto const& str : error_ignore) {
			if (log.msg.find(str) != std::string::npos) return;
		}
		notify("error");
	}
}

void handleBattery(const sensor_msgs::BatteryState& msg)
{
	for (auto const& voltage : msg.cell_voltage) {
		if (voltage < low_battery_threshold &&
		    voltage > 2.0) { // voltage < 2.0 likely indicates incorrect voltage measurement
			// notify low battery every time
			notify("low_battery");
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "led");
	ros::NodeHandle nh, nh_priv("~");

	nh_priv.param("blink_rate", blink_rate, 2.0);
	nh_priv.param("blink_fast_rate", blink_fast_rate, blink_rate * 2);
	nh_priv.param("fade_period", fade_period, 0.5);
	nh_priv.param("wipe_period", wipe_period, 0.5);
	nh_priv.param("flash_delay", flash_delay, 0.1);
	nh_priv.param("rainbow_period", rainbow_period, 5.0);

	nh_priv.param("notify/low_battery/threshold", low_battery_threshold, 3.7);
	nh_priv.param("notify/error/ignore", error_ignore, {});

	std::string led; // led namespace
	nh_priv.param("led", led, std::string("led"));
	if (!led.empty()) led += "/";

	ros::service::waitForService(led + "set_leds"); // cannot work without set_leds service
	set_leds_srv = nh.serviceClient<led_msgs::SetLEDs>(led + "set_leds", true);

	// wait for leds count info
	handleState(*ros::topic::waitForMessage<led_msgs::LEDStateArray>(led + "state", nh));

	auto state_sub = nh.subscribe(led + "state", 1, &handleState);

	auto set_effect = nh.advertiseService(led + "set_effect", &setEffect);

	auto mavros_state_sub = nh.subscribe("mavros/state", 1, &handleMavrosState);
	auto battery_sub = nh.subscribe("mavros/battery", 1, &handleBattery);
	auto rosout_sub = nh.subscribe("/rosout_agg", 1, &handleLog);

	timer = nh.createTimer(ros::Duration(0), &proceed, false, false);

	ROS_INFO("ready");
	notify("startup");
	ros::spin();
}
