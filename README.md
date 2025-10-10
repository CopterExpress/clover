
## Raspberry Pi image

Preconfigured image for Raspberry Pi with installed and configured software, ready to fly, is available [in the Releases section](https://github.com/CopterExpress/clover/releases).

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/CopterExpress/clover/build-image.yaml?branch=master)
![GitHub all releases](https://img.shields.io/github/downloads/CopterExpress/clover/total)

Image features:

* Raspbian Buster
* [ROS Noetic](http://wiki.ros.org/noetic)
* Configured networking
* [OpenCV](https://opencv.org/)
* [`mavros`](http://wiki.ros.org/mavros)
* Periphery drivers for ROS ([GPIO](https://clover.coex.tech/en/gpio.html), [LED strip](https://clover.coex.tech/en/leds.html), etc)
* `aruco_pose` package for marker-assisted navigation

API description for autonomous flights is available [on GitBook](https://clover.coex.tech/en/simple_offboard.html).

For manual package installation and running see [`clover` package documentation](clover/README.md).

## License

While the Clover platform source code is available under the MIT License, note, that the [documentation](docs/) is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.

**The main image for autonomous flight was taken from [Clover](https://github.com/CopterExpress/clover). We would like to express our special thanks to them. The nodes for the autonomous flight of PowerBelow were created by us personally.**
