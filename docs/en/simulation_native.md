# Native setup

Setting up the simulation environment from scratch requires some effort, but results in the most performant setup, with less chance of driver issues.

> **Hint** See up-to-date commands set for installation Clover simulation software in the script, that builds the virtual machine image with the simulator: [`install_software.sh`](https://github.com/CopterExpress/clover_vm/blob/master/scripts/install_software.sh).

Prerequisites: **Ubuntu 20.04**.

## Install ROS

Install ROS Noetic using the [official installation manual](http://wiki.ros.org/noetic/Installation/Ubuntu) (Desktop or Full install).

Add sourcing ROS' `setup.bash` initialization script to your `.bashrc`:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install required tools:

```bash
sudo apt install build-essential git python3-pip python3-rosdep
```

## Create a workspace for the simulation

Create a workspace for the simulation:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Clone Clover sources:

```bash
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/CopterExpress/clover
git clone --depth 1 https://github.com/CopterExpress/ros_led
git clone --depth 1 https://github.com/ethz-asl/mav_comm
```

Install all dependencies using `rosdep`:

```bash
cd ~/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Install Python dependencies:

```bash
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt
```

## Get PX4 sources

PX4 will be built along with the other packages in our workspace. You may clone it directly into the workspace or put it somewhere and symlink to `~/catkin_ws/src`. We will need to put its `sitl_gazebo` and `mavlink` submodules into `~/catkin_ws/src` as well.

Clone PX4 sources and make the required symlinks:

```bash
git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
ln -s ~/PX4-Autopilot ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/mavlink ~/catkin_ws/src/
```

> **Hint** You may use more recent PX4 version, but there would be more risk of something would not be working.

## Install PX4 prerequisites

PX4 comes with its own script for dependency installation. We may as well leverage it:

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh
```

This will install everything required to build PX4 and its SITL environment.

> **Hint** You may want to skip installing the ARM toolchain if you're not planning on compiling PX4 for your flight controller. To do this, use the `--no-nuttx` flag: `sudo ./ubuntu.sh --no-nuttx`.

Install more required Python packages:

```bash
pip3 install --user toml
```

## Add the Clover airframe

Add the Clover airframe to PX4 using the command:

```bash
ln -s ~/catkin_ws/src/clover/clover_simulation/airframes/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

## Install geographiclib datasets

`mavros` package requires geographiclib datasets to be present:

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

## Build the simulator

Build your workspace:

```bash
cd ~/catkin_ws
catkin_make
```

> **Note** If building fails with RAM issues (`c++: fatal error: Killed signal terminated program cc1plus`), reduce the number of parallel jobs using `-j` key. For example, to use only two parallel jobs use `catkin_make -j2` command.

## Run the simulator

In order to be sure that everything was built correctly, try running the simulator for the first time:

```bash
roslaunch clover_simulation simulator.launch
```

You can test autonomous flight using example scripts in `~/catkin_ws/src/clover/clover/examples` directory.

## Additional steps

Optionally, install roscore systemd service to have roscore running in background:

```bash
sed -i "s/pi/$USER/g" ~/catkin_ws/src/clover/builder/assets/roscore.service
sudo cp ~/catkin_ws/src/clover/builder/assets/roscore.service /etc/systemd/system
sudo systemctl enable roscore
sudo systemctl start roscore
```

Install any web server to serve Clover's web tools (`~/.ros/www` directory), e. g. Monkey:

```bash
wget https://github.com/CopterExpress/clover_vm/raw/master/assets/packages/monkey_1.6.9-1_amd64.deb -O /tmp/monkey_1.6.9-1_amd64.deb
sudo apt-get install -y /tmp/monkey_1.6.9-1_amd64.deb
sed "s/pi/$USER/g" ~/catkin_ws/src/clover/builder/assets/monkey | sudo tee /etc/monkey/sites/default
sudo -E sh -c "sed -i 's/SymLink Off/SymLink On/' /etc/monkey/monkey.conf"
sudo cp ~/catkin_ws/src/clover/builder/assets/monkey.service /etc/systemd/system/monkey.service
sudo systemctl enable monkey
sudo systemctl start monkey
```
