# Raspberry Pi image

The image contains:

* [Raspbian](https://www.raspberrypi.org/downloads/raspbian/) Stretch
* [ROS Kinetic](http://wiki.ros.org/kinetic)
* [A software package for Clever](https://github.com/CopterExpress/clever)

**The latest version of the image is available for [downloading from GitHub in section Releases](https://github.com/CopterExpress/clever/releases).**

> **Hint** The stable and supported version of the image is the release marked as **Latest release**.

<img src="../assets/image.png" width=400 alt="Download image">

## Installing the OS image on a MicroSD card

To install the image, you can use the [Etcher] utility (https://etcher.io).

[![Etcher](../assets/etcher.gif)](https://etcher.io)

## Image version

The version of the installed image may be found in file `/etc/clever_version`:

```bash
cat /etc/clever_version
```
