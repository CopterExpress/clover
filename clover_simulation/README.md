# `clover_simulation` ROS package

This package provides resources necessary for launching Gazebo simulation with Clover, along with `.launch` files for convenience.

## Launching the simulation

Simulation is launched by [`simulator.launch` file](launch/simulator.launch). This `.launch` file assumes that `px4` and `sitl_gazebo` packages are reachable from your current workspace.

The simulation may be configured by a set of arguments:

* `mav_id` (*integer*, default: *0*) - MAVLink identifier of the vehicle. **Note**: Multi-vehicle simulation is possible, but requires extensive changes to launch files;
* `est` (*string*, default: *lpe*, possible values: *lpe*, *ekf2*) - PX4 estimator selection. Note that this may be overriden in the startup scripts for your craft;
* `vehicle` (*string*, default: *clover*) - PX4 vehicle name. Depending on this parameter, different PX4 presets will be loaded. **Note**: The default value, *clover*, requires you to use [Clover-specific PX4 branch](https://github.com/CopterExpress/Firmware/tree/v1.10.1-clever);
* `main_camera` (*boolean*, default: *true*) - controls whether the drone will have a vision position estimation camera;
* `rangefinder` (*boolean*, default: *true*) - controls whether the drone will have a laser rangefinder;
* `led` (*boolean*, default: *true*) - controls whether the drone will have a programmable LED strip;
* `gps` (*boolean*, default: *true*) - controls whether the drone will have a simulated GPS module;

In order to start the simulation, run:

```bash
roslaunch clover_simulation simulator.launch
```

This will start a new Gazebo instance (using `gazebo_ros` package), load a PX4 SITL instance, spawn a Clover model and start Clover ROS nodes. The PX4 console will be accessible in the terminal where `roslaunch` was performed.

### Changing simulation speed (PX4 1.9+)

In order to run simulation faster or slower than realtime, use the `PX4_SIM_SPEED_FACTOR` environment variable, [as stated in the PX4 docs](https://dev.px4.io/v1.9.0/en/simulation/#simulation_speed).

If `PX4_SIM_SPEED_FACTOR` is not set, it is assumed that it is equal to 1.0.

Note that Gazebo may slow the simulation down automatically. This may not be handled gracefully, so if you notice Gazebo's "Real Time Factor" being significantly lower than your `PX4_SIM_SPEED_FACTOR`, be sure to adjust it accordingly.

### Changing initial world

By default, the `simulator.launch` file will start the simulation with [`resources/worlds/clover.world`](resources/worlds/clover.world) as its base world. Note that the `real_time_update_rate` is set to 250 - this is required for PX4 lockstep simulation to work correctly.

If you wish to create your own world for the simulation, be sure to derive it from `clover.world` to avoid issues with PX4 plugins.

You may set the world name in `simulator.launch` as the `world_name` parameter for `gazebo_ros` instance.

### Configuring the vehicle

`simulator.launch` utilizes the same `clover.launch` file from the `clover` ROS package, so ROS node reconfiguration is the same as on the real drone.

PX4 may be reconfigured using QGroundControl, just like a real drone. Some parameters may require rebooting the drone, which is performed by shutting the simulated environment down and restarting it.

PX4 will write its parameters and logs to `${ROS_HOME}/eeprom/parameters` and `${ROS_HOME}/log`, respectively. Note that the log directory naming schema for PX4 logs is different from ROS: PX4 creates log directories based on the current date, which makes them relatively simple to find.

## LED plugin (sim_leds)

A visual Gazebo plugin is used for the LED strip. An example of the plugin usage is provided in [`led_strip.xacro`](../clover_description/urdf/leds/led_strip.xacro).

The plugin accepts the following parameters during instantiation:

* `robotNamespace` (*string*, default: "") - a ROS namespace for the plugin;
* `ledCount` (*integer*, required) - total number of LEDs in a strip.

The plugin will provide the following service:

`led/set_leds` ([*led_msgs/SetLEDs*](https://github.com/CopterExpress/ros_led/blob/v0.0.6/led_msgs/srv/SetLEDs.srv)) - set the LED colors to the provided values.

The plugin will provide the following topics:

`led/state` ([*led_msgs/LEDStateArray*](https://github.com/CopterExpress/ros_led/blob/v0.0.6/led_msgs/msg/LEDStateArray.msg)) - current LED strip state.

Other nodes are not expected to write to `led/state` topic.

All provided topics and services will be namespaced according to the `robotNamespace` parameter.

## Throttling camera plugin (throttling_camera)

By default, Gazebo camera sensors will use their `update_rate` parameter as an upper bound for the actual rate. This may result in much lower rates than expected. This may be fine for object recognition tasks where the camera is not the primary positioning sensor, but is not desirable in our case, when the camera is used for position calculation.

We provide a Gazebo-ROS plugin for the camera sensor that will throttle down the simulation to maintain update rate. The plugin API is based on the `gazebo_ros_camera` plugin, and respects the following parameters in SDF:

* `<minUpdateRate>` (*double*, default: same as `<updateRate>`) - least allowed publish/update rate for the camera (in Hz);
* `<windowSize>` (*integer*, default: 10) - number of last update intervals that are considered for throttling;
* `<maxStDev>` (*double*, default: 0.02) - maximum standard deviation value for update intervals.

The simulation will be slowed down if the average update rate (averaged over `<windowSize>` samples) is lower than `<minUpdateRate>` and is consistent (standard deviation over `<windowSize>` samples is less than `<maxStDev>`).
