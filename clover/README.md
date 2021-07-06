# `clover` ROS package

A bundle for autonomous navigation and drone control.

## Manual installation

Install ROS Noetic according to the [documentation](http://wiki.ros.org/noetic/Installation), then [create a Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Clone this repo to directory `~/catkin_ws/src/clover`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/CopterExpress/clover.git clover
```

All the required ROS packages (including `mavros` and `opencv`) can be installed using `rosdep`:

```bash
cd ~/catkin_ws/
rosdep install -y --from-paths src --ignore-src
```

Build ROS packages (on memory constrained platforms you might be going to need to use `-j1` key):

```bash
cd ~/catkin_ws
catkin_make -j1
```

To complete `mavros` install you'll need to install `geographiclib` datasets:

```bash
curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash
```

You may optionally install udev rules to provide `/dev/px4fmu` symlink to your PX4-based flight controller connected over USB. Copy `99-px4fmu.rules` to your `/lib/udev/rules.d` folder:

```bash
cd ~/catkin_ws/src/clover/clover/udev
sudo cp 99-px4fmu.rules /lib/udev/rules.d
```

Alternatively you may change the `fcu_url` property in `mavros.launch` file to point to your flight controller device.

## Running

To start connection to the flight controller, use:

```bash
roslaunch clover clover.launch
```

For the simulation information see the [corresponding article](https://clover.coex.tech/en/simulation.html).

> Note that the package is configured to connect to `/dev/px4fmu` by default (see [previous section](#manual-installation)). Install udev rules or specify path to your FCU device in `mavros.launch`.
