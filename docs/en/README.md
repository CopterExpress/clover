Clever
======

<img src="../assets/clever3.png" align="right" width="300px" alt="Klever">

CLEVER (Russian: *"Клевер"*, meaning *"Clover"*) is an educational kit of a programmable quadcopter that consists of popular open source components, and a set of necessary documentation and libraries for working with it.

The kit includes a Pixhawk/Pixracer flight controller with the PX4 flight stack, a Raspberry Pi 3 as a controlling onboard computer, and a camera module for performing flights with the use of computer vision, and a set of various sensors and other peripherals.

The same platform was used for creating many "large" projects of the Copter Express company, for example, drones for [autonomous pizza delivery promotion events](https://www.youtube.com/watch?v=hmkAoZOtF58) (Samara, Kazan); a drone for coffee delivery at the Skolkovo Innovation Center, a autonomous quadcopter with charging station for site monitoring and security, winning drones in [Robocross-2016](https://www.youtube.com/watch?v=dGbDaz_VmYU), [Robocross-2017](https://youtu.be/AQnd2CRczbQ)" competitions, and many others.

To learn how to build, configure, pilot and program a Clever autonomous drone, use this gitbook.

If you have studied our gitbook, but have not found an answer to your question, write to our support chat, and our specialists will be happy to answer you: https://t.me/COEXHelpdesk.

We also have a chat for programmers coding for PX4, autonomous navigation indoors, and drone swarms https://t.me/DroneCode.

Raspberry Pi image
----------------------

**Preconfigured OS** image for Raspberry Pi 3 with installed and configured software is available [here](microsd_images.md).

The image includes:

* Raspbian Stretch
* ROS Kinetic
* Configured [networking](network.md)
* OpenCV
* mavros
* A software set for working with Clever

[API description](simple_offboard.md) for autonomous flights.

The source code of the collector of the image and only can be found at [GitHub](https://github.com/CopterExpress/clever).
