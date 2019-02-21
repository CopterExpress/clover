PX4 Simulation
===

Main article: https://dev.px4.io/en/simulation/

PX4 simulation is possible in Linux and macOS with the use of physical environment simulation systems [jMavSim](https://pixhawk.org/dev/hil/jmavsim) and [the Gazebo](http://gazebosim.org).

jMavSim is a lightweight environment intended only for testing multi-rotor aircraft systems; Gazebo is a versatile environment for all types of robots.

Launching PX4 SITL
--

1. Clone repository from PX4.

```(bash)
git clone https://github.com/PX4/Firmware.git
cd Firmware
```

jMavSim
--

Main article: https://dev.px4.io/en/simulation/jmavsim.html

For simulation using the jMavSim lightweight environment, use the following command:

```(bash)
make posix_sitl_default jmavsim
```

To use the LPE position calculation module instead of EKF2, use:

```(bash)
make posix_sitl_lpe jmavsim
```

Gazebo
--

Main article: https://dev.px4.io/en/simulation/gazebo.html

To get started, install Gazebo 7. On a Mac:

```(bash)
brew install gazebo7
```

On Linux (Debian):

```(bash)
sudo apt-get install gazebo7 libgazebo7-dev
```

Start simulation from the Firmware folder:

```(bash)
make posix_sitl_default gazebo
```

You can run a simulation in headless mode (without a window client). To do this, use the following command:

```(bash)
HEADLESS=1 make posix_sitl_default gazebo
```

Connection
---

QGroundControl will automatically connect to the running simulation on startup. The operation will be the same as, as in the case of a real flight controller.

To connect MAVROS to the simulation, use the UDP Protocol, a local IP address, and port 14557, for example:

```(bash)
roslaunch mavros px4.launch fcu_url:=udp://@127.0.0.1:14557
```
