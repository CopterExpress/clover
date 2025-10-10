
## Raspberry Pi image

Preconfigured image for Raspberry Pi with installed and configured software, ready to fly, is available [in the Releases section](https://github.com/CopterExpress/clover/releases).

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/CopterExpress/clover/build-image.yaml?branch=master)
![GitHub all releases](https://img.shields.io/github/downloads/CopterExpress/clover/total)

Image features:

* Raspbian Buster
* [ROS Noetic](http://wiki.ros.org/noetic)
* Configured networking
* OpenCV
* [`mavros`](http://wiki.ros.org/mavros)
* Periphery drivers for ROS ([GPIO](https://clover.coex.tech/en/gpio.html), [LED strip](https://clover.coex.tech/en/leds.html), etc)
* `aruco_pose` package for marker-assisted navigation
* `clover` package for autonomous drone control

API description for autonomous flights is available [on GitBook](https://clover.coex.tech/en/simple_offboard.html).

For manual package installation and running see [`clover` package documentation](clover/README.md).

## Support

[![Telegram Support Chat](https://img.shields.io/endpoint?label=Support%20Chat&url=https%3A%2F%2Ftelegram-badge-4mbpu8e0fit4.runkit.sh%2F%3Furl%3Dhttps%3A%2F%2Ft.me%2FCOEXHelpDesk)](https://t.me/COEXHelpdesk)

## License

While the Clover platform source code is available under the MIT License, note, that the [documentation](docs/) is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
