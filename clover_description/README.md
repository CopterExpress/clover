# `clover_description` ROS package

This package contains models and URDF descriptions for the Clover 4 drone. These descriptions can be used for Gazebo simulation environment.

Note that in order to use these descriptions in Gazebo, you need to use the plugins from [PX4 `sitl_gazebo` package](https://github.com/PX4/sitl_gazebo) and `clover_simulation` package.

## Usage

The descriptions are provided as [`xacro`-enabled](https://wiki.ros.org/xacro) URDF files. A [`spawn_drone.launch`](launch/spawn_drone.launch) file that spawns the model in a running Gazebo instance is also provided for convenience.

You may provide additional parameters for `spawn_drone.launch` as well:

* `main_camera` (*boolean*, default: *true*) - controls whether the drone will have a downward-facing camera attached;
* `rangefinder` (*boolean*, default: *true*) - controls whether the drone will have a downward-facing laser rangefinder attached;
* `led` (*boolean*, default: *true*) - controls whether the drone will have a programmable LED strip (requires plugins from `clover_simulation`);
* `gps` (*boolean*, default: *true*) - controls whether the drone will have a simulated GPS attached (requires plugins from `sitl_gazebo`);
* `maintain_camera_rate` (*boolean*, default: *false*) - slow down the simultion to maintain camera publishing rate (internally changes the camera plugin from `libgazebo_ros_camera.so` to `libthrottling_camera.so` from [`clover_simulation`](../clover_simulation/README.md#throttling-camera-plugin-throttling_camera)).

For example, in order to spawn a drone without a GPS module, you may use the following command:

```bash
roslaunch clover_description spawn_drone.launch gps:=false
```

## Tweaking

By default, the `spawn_drone.launch` command will use the [`clover4.xacro` description file](urdf/clover/clover4.xacro). This is a "high-level" description of the drone, mainly used to attach additional sensors.

The drone "physics" may be tweaked by changing the [`clover4_physics.xacro` file](urdf/clover/clover4_physics.xacro).
