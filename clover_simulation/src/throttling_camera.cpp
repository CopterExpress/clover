#include <gazebo/plugins/CameraPlugin.hh>
// physics definitions and functions: Required to perform slowdown
#include <gazebo/physics/physics.hh>
// ROS simulated camera class with most of the boilerplate already in place
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <ros/ros.h>
#include <memory>

#include <deque>
#include <algorithm>

namespace
{
/**
 * Simple statistics-collecting class.
 */
class AverageStat
{
private:
	/** Currently collected samples. */
	std::deque<double> samples;
	/** Number of samples to store (also, the number of samples considered "adequate") */
	size_t maxSamples;
	/** Currently stored average value */
	double average = 0;
	/** Currently stored standard deviation value */
	double stdev = 0;
	/** Largest standard deviation that is considered adequate */
	double maxStDev;

public:
	AverageStat(size_t numSamples, double validStDev) :
		samples(),
		maxSamples(numSamples),
		maxStDev(validStDev)
	{}

	/**
	 * Add a sample and recalculate average and standard deviation.
	 * 
	 * @param sample New sampled value.
	 * @return New average value.
	 */
	double update(double sample)
	{
		samples.push_back(sample);
		if (samples.size() > maxSamples)
		{
			samples.pop_front();
		}
		average = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
		double stdevSquared = std::accumulate(samples.begin(), samples.end(), 0.0,
			[&](double sum, double xi) {
				return sum + (xi - average) * (xi - average);
			}) / samples.size();
		stdev = std::sqrt(stdevSquared);
		return average;
	}

	/**
	 * Get current average value of all samples.
	 * 
	 * @note This function will return a result even if it is not considered valid.
	 */
	double getAverage() const
	{
		return average;
	}

	/**
	 * Get current standard deviation of all samples.
	 * 
	 * @note This function will return a result even if it is not considered valid.
	 */
	double getStDev() const
	{
		return stdev;
	}

	/**
	 * Check if the average value is considered "adequate".
	 * 
	 * @return True if the number of samples is not less than required and standard deviation is within limits, false otherwise
	 */
	bool isAdequate() const
	{
		return (samples.size() >= maxSamples) && (stdev < maxStDev);
	}

	/**
	 * Drop all samples and start anew.
	 * 
	 * @note This does not actually change average and stdev, but the stats are considered inadequate after this call.
	 */
	void reset()
	{
		samples.clear();
	}
};

}

namespace throttling_camera
{

/**
 * Gazebo camera plugin for a world-throttling (rate-preserving) camera.
 * 
 * This plugin will slow the simulation down to maintain average publishing rate. Note that
 * this plugin will *only* perform slowdown, it *will not* speed the simulation back up!
 */
class ThrottlingCamera : public gazebo::CameraPlugin, gazebo::GazeboRosCameraUtils
{
private:
	/** A pointer to the Gazebo camera sensor. */
	gazebo::sensors::SensorPtr camPtr;
	/** A pointer to the current simulated world (required to change world parameters) */
	gazebo::physics::WorldPtr world;
	/** A pointer to the physics preset manager. Used to actually slow the simulation down. */
	gazebo::physics::PresetManagerPtr presetManager;

	/** Maximum update interval that is considered "okay". Should be higher than the "average" update interval to avoid extreme slowdowns */
	double maxUpdateInterval;

	/** Statistics for publishing time intervals. */
	std::unique_ptr<AverageStat> timeSamples;
public:
	ThrottlingCamera() = default;
	~ThrottlingCamera() override = default;

	/**
	 * Plugin load function. Called by Gazebo each time the plugin is instantiated.
	 * 
	 * @param parent Gazebo sensor that this plugin connects to.
	 * @param sdf SDF element containing this plugin.
	 */
	void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) override
	{
		if (!ros::isInitialized())
		{
			ROS_FATAL_NAMED("throttling_camera", "ROS node for Gazebo has not been initialized, unable to load plugin");
			return;
		}
		ROS_DEBUG_NAMED("throttling_camera", "Initializing ROS throttling (stable-rate) camera");

		CameraPlugin::Load(parent, sdf);

		world = gazebo::physics::get_world(parent->WorldName());
#if GAZEBO_MAJOR_VERSION >= 8
		presetManager = world->PresetMgr();
#else
		presetManager = world->GetPresetManager();
#endif /* GAZEBO_MAJOR_VERSION */

		// Same as in PX4
		if (presetManager->CurrentProfile() != "default_physics")
		{
			gzwarn << "Current physics profile is not default_physics, but actually is " << presetManager->CurrentProfile() << "\n";
			if (!presetManager->CurrentProfile("default_physics"))
			{
				gzerr << "Could not set current profile to default_physics!\n";
			}
		}

		double minUpdateRate = parent->UpdateRate();
		if (sdf->HasElement("minUpdateRate"))
		{
			minUpdateRate = sdf->Get<double>("minUpdateRate");
		}
		maxUpdateInterval = 1.0 / minUpdateRate;

		size_t windowSize = 10;
		if (sdf->HasElement("windowSize"))
		{
			windowSize = sdf->Get<size_t>("windowSize");
		}

		double maxStDev = 0.02;
		if (sdf->HasElement("maxStDev"))
		{
			maxStDev = sdf->Get<double>("maxStDev");
		}

		timeSamples.reset(new AverageStat(windowSize, maxStDev));

		camPtr = parent;

		parentSensor_ = camPtr;
		width_ = width;
		height_ = height;
		depth_ = depth;
		format_ = format;
		camera_ = camera;

		gazebo::GazeboRosCameraUtils::Load(parent, sdf);
	}

	/**
	 * Frame callback. Called every time a new frame is rendered by the camera.
	 * 
	 * Checks whether we should slow simulation down and publishes a new image
	 * message.
	 * 
	 * @param image Image data.
	 * @param width Image width, in pixels.
	 * @param height Image height, in pixels.
	 * @param depth Image depth, in bytes.
	 * @param format Image format description string.
	 */
	void OnNewFrame(const unsigned char *image, unsigned int width, unsigned int height, unsigned int depth,
	                const std::string &format) override {

		// Note: sensorUpdateTime uses simulated time
		auto sensorUpdateTime = camPtr->LastMeasurementTime();
		// If sensor was not active for some reason, we allow it to get new data on next frame
		if (!camPtr->IsActive())
		{
			camPtr->SetActive(true);
			last_update_time_ = sensorUpdateTime;
			timeSamples->reset();
			return;
		}

		boost::mutex::scoped_lock lock(*image_connect_count_lock_);
		if (*image_connect_count_ > 0)
		{
			if (sensorUpdateTime < last_update_time_)
			{
				ROS_WARN_NAMED("throttling_camera", "Negative sensor update time difference (world reset?)");
				last_update_time_ = sensorUpdateTime;
				timeSamples->reset();
			}

			auto timeDelta = sensorUpdateTime - last_update_time_;
			timeSamples->update(timeDelta.Double());

			// We want to throttle the simulation down if we have measurements too far apart
			if (timeSamples->isAdequate() && timeSamples->getAverage() > maxUpdateInterval)
			{
				ROS_INFO_STREAM_NAMED("throttling_camera", "Had average update period of "
										<< timeSamples->getAverage() << " (stdev: " << timeSamples->getStDev() <<  ")"
										<< ", but desired update period is " << update_period_
										<< ", throttling simulation down");
				boost::any currentRealTimeUpadteRateParam;
				if (!presetManager->GetCurrentProfileParam("real_time_update_rate", currentRealTimeUpadteRateParam))
				{
					gzerr << "Failed to get real time update rate parameter!\n";
				}
				auto currentRate = boost::any_cast<double>(currentRealTimeUpadteRateParam);
				// We are being somewhat aggressive here, maybe we could throttle the world
				// down in steps?
				double slowdownFactor = update_period_ / timeSamples->getAverage();
				auto nextRate = currentRate * slowdownFactor;
				if (!presetManager->SetCurrentProfileParam("real_time_update_rate", nextRate))
				{
					gzerr << "Failed to set real time update rate parameter!\n";
				}
				if (slowdownFactor < 0.5)
				{
					ROS_WARN_STREAM_NAMED("throttling_camera", "Simulation slowed down significantly; consider running"
					                      "the simulation with a lower PX4_SIM_SPEED_FACTOR value (slowed down from "
					                      << currentRate << " to " << nextRate << " updates per second)");
				}
				// We're discarding old samples to avoid extensive slowdown
				timeSamples->reset();
			}
			PutCameraData(image, sensorUpdateTime);
			PublishCameraInfo(sensorUpdateTime);
		}
		last_update_time_ = sensorUpdateTime;
	}
};
}

GZ_REGISTER_SENSOR_PLUGIN(throttling_camera::ThrottlingCamera);
