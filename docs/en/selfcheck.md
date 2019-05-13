# Automatic check

Before flying (especially autonomous), you can use several methods of automatic self-testing of the quadcopter subsystems.

## <span>selfcheck</span>.py

Utility `selfcheck.py` is part of `clever` package, and automatically tests the main aspects of the ROS platform and the PX4. The utility is pre-installed on [the Raspberry Pi image](microsd_images.md).

To initiate it, type in [the Raspberry Pi console](ssh.md):

```(bash)
rosrun clever selfcheck.py
```

<img src="../assets/selfcheck.png">

Description of some checks:

* FCU – checking proper connection with the flight controller;
* IMU – checking correctness of the data from IMU;
* Local position – presence of the local position of the drone;
* Velocity estimation – drone velocity estimation (**if this check fails, never take off offline!**);
* Global position (GPS) — presence of the global position (GPS required);
* Camera — proper operation of the Raspberry camera.

## commander check

To check the main sub systems of PX4 and the possibility of arming at the moment, you can perform command `commander check` in the MAVLink console.

<img src="../assets/commander-check.png">

When using SITL instead of the MAVLink console, use a terminal with SITL running.
