# Gazebo for MAVLink SITL and HITL

[![Build Status](https://travis-ci.org/PX4/sitl_gazebo.svg?branch=master)](https://travis-ci.org/PX4/sitl_gazebo)

This is a flight simulator for multirotors, VTOL and fixed wing. It uses the motor model and other pieces from the RotorS simulator, but in contrast to RotorS has no dependency on ROS. This repository is in the process of being re-integrated into RotorS, which then will support ROS and MAVLink as transport options: https://github.com/ethz-asl/rotors_simulator.

**If you use this simulator in academic work, please cite RotorS as per the README in the above link.**


## Installation (Gazebo 9)

Follow instructions on the [official site](http://gazebosim.org/tutorials?cat=install) to install Gazebo.

### Ubuntu

```bash
sudo apt-get install gazebo9 libgazebo9-dev
```

### Mac OS

```bash
brew tap osrf/simulation
brew install gazebo9
```

### Arch Linux

```bash
sudo packer -S gazebo
# or
yaourt -S gazebo
```


## *sitl_gazebo* plugin dependencies

Some plugins on this packages require some specific dependencies:

* Protobuf is required to generate custom protobuf messages to be published and subscribed between topics of different plugins;
* Jinja 2 is used to generate some SDF models from templates;
* Gstreamer is required for a plugin that streams video from a simulated camera.


### Ubuntu 

```bash
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
```

#### Gstreamer:
```
sudo apt-get install $(apt-cache --names-only search ^gstreamer1.0-* | awk '{ print $1 }' | grep -v gstreamer1.0-hybris) -y
```


### Mac OS

```bash
pip install rospkg jinja2
brew tap homebrew/versions
brew install eigen graphviz libxml2 sdformat3 opencv glib
brew install homebrew/versions/protobuf260
```

#### Gstreamer:
```
brew install gstreamer gst-plugins-base gst-plugins-good
```

### Arch Linux

```bash
sudo pacman -S --noconfirm --needed eigen3 hdf5 opencv protobuf vtk yay python2-jinja
```

#### Gstreamer:
```bash
sudo pacman -S --needed $(pacman -Ssq gstreamer)
```


## Build *sitl_gazebo*

Clone the repository to your computer.

**IMPORTANT: If you do not clone to ~/src/sitl_gazebo, all remaining paths in these instructions will need to be adjusted.**

```bash
mkdir -p ~/src
cd src
git clone --recursive https://github.com/PX4/sitl_gazebo.git
```

Create a build folder in the top level of your repository:

```bash
mkdir build
```

Navigate into the build directory and invoke CMake from it:

```bash
cd ~/src/sitl_gazebo
cd build
cmake ..
```

Now build the gazebo plugins by typing:

```bash
make -j$(nproc) -l$(nproc)
```

Next add the location of this build directory to your gazebo plugin path, e.g. add the following line to your `.bashrc` (Linux) or `.bash_profile` (Mac) file:

```bash
# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/src/sitl_gazebo/build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/src/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
```

You also need to add the the root location of this repository, e.g. add the following line to your `.bashrc` (Linux) or `.bash_profile` (Mac) file:

```bash
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=$HOME/src/sitl_gazebo
```


### Geotagging Plugin
If you want to use the geotagging plugin, make sure you have `exiftool` installed on your system. On Ubuntu it can be installed with:

```
sudo apt-get install libimage-exiftool-perl
```


## Install

If you wish the libraries and models to be usable anywhere on your system without
specifying th paths, install as shown below.

**Note: If you are using Ubuntu, it is best to see the packaging section.**

```bash
sudo make install
```


## Testing

Gazebo will now launch when typing 'gazebo' on the shell:

```bash
. /usr/share/gazebo/setup.sh
. /usr/share/mavlink_sitl_gazebo/setup.sh
gazebo worlds/iris.world
```

Please refer to the documentation of the particular flight stack how to run it against this framework, e.g. [PX4](http://dev.px4.io/simulation-gazebo.html)


### Unit Tests

For building and running test an installation of 'googletest' is needed.

On Ubuntu it can be installed with:

```bash
sudo apt-get install libgtest-dev
cd /usr/src/googletest
sudo cmake . && cd googletest
sudo make -j$(nproc) -l$(nproc)
sudo cp *.a /usr/lib
```

On macOS it needs to be installed from source:

```bash
git clone https://github.com/google/googletest
pushd googletest
mkdir build
pushd build
cmake ..
make -j$(nproc) -l$(nproc)
make install
```

When writing test it’s important to be careful which API functions of Gazebo are called. As no Gazebo server is running during the tests some functions can produce undefined behaviour (e.g. segfaults).


#### *catkin tools*

With *catkin*, the unit tests are enabled by default.

```bash
# After setting up the catkin workspace
catkin build -j4 -l4 -DBUILD_ROS_INTERFACE=ON
cd build/mavlink_sitl_gazebo/
catkin run_tests
```

#### Plain CMake

For building the tests with plain CMake, the flag `ENABLE_UNIT_TESTS` needs to be provided.

```bash
mkdir build && cd build
cmake -DENABLE_UNIT_TESTS=On ..
```

Then build and run the tests:

```bash
make -j$(nproc) -l$(nproc)
make test
```


## Packaging

### Debian packages

To create a debian package for Ubuntu and install it to your system.

```bash
cd Build
cmake ..
make
rm *.deb
cpack -G DEB
sudo dpkg -i *.deb
```
