# Raspberry Pi image

The RPi image for Clever contains all the necessary software for working with Clever and [programming autonomous flights](simple_offboard.md). The Clever platform is based on [Raspbian OS](https://www.raspberrypi.org/downloads/raspbian/) and robotics framework [ROS](ros.md). The source code of the image builder and all the additional packages is [available on GitHub](https://github.com/CopterExpress/clever).

## Usage

1. Download the latest stable release of the image – **<a class="latest-image" href="https://github.com/CopterExpress/clever/releases">download</a>**.
2. Download and install [Etcher](https://www.balena.io/etcher/), the software for flashing images (available for Windows/Linux/macOS).
3. Put the MicroSD-card into your computer (use an adapter if necessary).
4. Flash the downloaded image to the card using Etcher.
5. Put the card into the Raspberry Pi.

<img src="../assets/etcher.png" class="zoom">

After flashing the image on the MicroSD-card, you can [connect to the Clever over Wi-Fi](wifi.md), use [wireless connection in QGroundControl](gcs_bridge.md), gain access to the Raspberry [over SSH](ssh.md) and use all the other features.

**Next:** [Connecting over Wi-Fi](wifi.md).
