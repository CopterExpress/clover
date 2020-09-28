# Positioning systems setup

The `clover` software officially supports the following [positioning systems](../programming.md) :

* [optical flow](../optical_flow.md).
* [aruco](../aruco.md).
* [gps](../gps.md).

The `clever-show` software supports all positioning systems supporte by`clover`.

>Following configuration examples are for the Clover 4 copter, assembled and configured according to the [documentation](../README.md).

It is recommended to set up and check one copter from the group and reproduce its settings for the rest of the copters before starting the group.

Configure one copter to work with any of listed above positioning systems. Setup may include the following steps:

* Editing `.launch` files of ROS package `clover`. These files are located in the directory `/home/pi/catkin_ws/src/clover/clover/launch` on the copter (on Raspberry Pi).
* Configuring the flight controller parameters.
* Editing [client](client.md) `clever-show` configuration file.
* Editing [server](server.md) `clever-show` configuration file.
* [Calibrating camera](../camera_calibration.html).

Make sure that the copter holds the position autonomously: mark the checkbox near the name of the copter and press the "Takeoff" button in the right panel of the server interface. The copter should take off at the height specified in the `takeoff_height` parameter of the "FLIGHT" section in the [client configuration](https://github.com/CopterExpress/clever-show/blob/master/drone/config/spec/configspec_client.ini). By default, this height is 1 meter. If the copter takes off and holds a position at 1 meter height, the check is passed. Put the copter on the ground by pressing the `Land` or `Land All` button. **Warning!** For your safety it is recommended to perform a test of autonomous takeoff with the remote control turned on and with ability to intercept the copter into the manual mode.

If the takeoff was successful, reproduce the client configuration, positioning settings and flight controller parameters to the other copters. Select the successfully configured copter in the table and upload the necessary configuration files from the selected copter to your computer:

* `.launch` files for `clover` configuration can be saved with the command `Selected drones -> Retrieve file` in the server application. In the dialog box that opens, enter the path to the file on the copter: `launch` files of `clover` are located in `/home/pi/catkin_ws/src/clover/clover/launch/`, to this path you need to add the desired name of the `.launch` file, e.g. `/home/pi/catkin_ws/src/clover/launch/clover.launch`. After clicking the `OK` button, a new dialog box will open with the choice of the path for saving the specified file on your computer.
* The copter configuration file (`.ini`) can be saved by right-clicking on the row with the configured copter, selecting `Edit config` from the drop-down menu and then clicking the button `Save as`. You can also drag a cell from the `configuration` column to the file manager of your system - the server will automatically copy the configuration file to the open directory of the file manager.
* The server configuration file (`.ini`) can be saved by selecting `Server -> Edit server config` from the top menu and then clicking the button `Save as`.
* The flight controller configuration file can be saved by connecting to the flight controller through [QGroundControl](http://qgroundcontrol.com) application. It is possible to connect directly to the flight controller [via USB port](../connection.md) or  [via TCP or UDP bridge](../gcs_bridge.md)  (the TCP bridge is configured in the `clever-show` image by default,  in the `Host Address` field you can enter the name of the copter instead of the ip address with the addition of .local at the end, e.g. clover-1.local). After connection you should go to [section](https://docs.px4.io/master/en/advanced_config/parameters.html#tools) `Parameters -> Tools -> Save to file...` and choose the path to save the parameter file.
* The camera calibration file is useful for refinement of visual positioning. The name of the calibration file should consist of the id of the copter, for which the calibration was made, with the addition of the extension `.yaml`, e.g.`clover-1.yam`. To get the calibration files, use the [manual](../camera_calibration.md).

After loading the necessary files from the configured copter, copy these files to the other copters: select the necessary copters in the table and use the commands `Send -> Configuration`, `Send -> Launch files folder`, `Send -> FCU parameters file`, `Send -> Camera calibrations` from the [`Selected drones`](server.md#selected-drones-section) section of the server application .

## Settings of clever-show server and client

The `clever-show` software suite includes many status checks of the copters to minimize the number of failed launches as well as a set of parameters to configure the positioning systems. All settings are stored in client and server application configuration files. Each positioning system has its own features that need to be taken into account when configuring the server-client interaction. Below are the settings that you need to pay attention to when configuring the client and server:

* Server:
  * [CHECKS](server.md#checks-section) section - server side copter telemetry checks

* Client:
  * [FLIGHT](client.md#flight-section) section - name of the frame_id reference coordinate system, flight parameters
  * [FLOOR FRAME](client.md#floor-frame section) section - allows to create a new coordinate system with the name `floor` in relation to any existing coordinate system:
    * `map` - matches the starting position of the copter when using optical flow or gps
    * `aruco_map` - matches the origin of ArUco marker map coordinates
    * `gps` - the origin of coordinates is in the specified GPS coordinate with rotation by the specified angle relative to the north, it is adjusted in [GPS FRAME] (client.md#gps-frame-section) and allows to set the coordinate system with a common origin for all copters.
  * [FAILSAFE](client.md#failsafe-section) section - disabled by default, but allows you to configure emergency landing conditions of the copter:
    * at loss of visual position - useful for ArUco marker positioning system.
    * if there is a large distance between the current position and the point where the copter should be - with the help of this check it is possible to avoid unexpected behavior of copters during collisions or any physical problems
  * [EMERGENCY LAND](client.md#emergency-land-section) section - configures emergency landing of the copter: the parameter `thrust` sets the throttle of motors to start landing, after the time `decrease_thrust_after` the copter starts to gradually reduce the throttle level to 0. **Warning!** The default gas level of the emergency landing is 45% - this setting works for the Clover 4 copter with a 3S battery. If your configuration is different, you should first determine the hovering throttle and then set the `thrust` parameter to 5% less than the hovering throttle. If the emergency landing throttle exceeds the hovering throttle of the copter, the copter may fly up for the first 3 seconds (the default value of `decrease_thrust_after`) and only then will start to smoothly reduce the motor throttle to 0.

## Optical flow

Optical flow - a method of positioning by computation of camera movement speed by calculation of pixel shift between adjacent frames.

`Optical flow` is suitable for a demonstration flight of one copter or for a synchronous flight of several copters following the same indoor trajectory. But it is necessary to take into account, that this coordinate system doesn't set general origin for all copters - the origin of coordinates is the start position of each particular copter. Also it is necessary to note that this method calculates the position of the copter by speed of moving of its camera - it means, that an error constantly accumulates in the copter position and the position becomes more inaccurate with time. Therefore, it is not recommended to use this coordinate system for complex and long group flights.

The `clever-show` image for the copter is set by default for flight on this positioning system (as well as the `clover` image) - the copter should have a laser rangefinder mounted on it and the camera should be tilted down by the plume back.

Clover ROS package, configured by default, provides autonomous flight using `optical flow`. The parameters loaded by default when loading the firmware adapted for clover 4 involves positioning with `optical flow`. The default client and server configuration files also do not need to be modified to fly through this positioning system.

However, if you need to switch to 'optical flow' positioning system from another previously configured system or to change the settings of this system, the information below could help.

All the configuration files for `optical flow` configuration are located in the  [examples/positioning/optical flow](https://github.com/CopterExpress/clever-show/tree/master/examples/positioning/optical%20flow) folder.

### Clover ROS package configuration

Configuring `optical flow` positioning system is described in the `clover` [documentation](../optical_flow.md).

An example of a `.launch` configuration file: [clover.launch](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/optical%20flow/launch/clover.launch).

### Flight controller configuration

Параметры, настраивающие полётный контроллер на возможность полёта по `optical flow`: [optical_flow.params](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/optical%20flow/optical_flow.params).

Для загрузки параметров на выделенные в таблице коптеры воспользуйтесь командой `Selected drones -> Send -> FCU parameters file` из верхнего меню и укажите путь к файлу с параметрами полётного контроллера.

### Client configuration

#### Option 1: Loading relevant parameters from the configuration example

Client configuration with values applicable to work with `optical flow`: [client.ini](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/optical%20flow/client.ini).

To upload, use the command `Selected drones -> Send -> Configuration` from the top menu of the server. In the opened dialog box, select the option `Modify` and specify the path to the file `client.ini`.

#### Option 2. Reset all client configurations

In order to configure the client configuration for positioning by optical flow, you can reset the configuration data to default values. To do this, you need to delete the generated configuration files on the clients:

* Select in the table the copter or copters which configuration you want to reset.
* Execute the command to delete the configurations on the selected copters by selecting `Selected drones -> Send -> Command`. In the dialog box that opens, type the command `rm config/client.ini` and press `OK`.

### Server configuration

#### Option 1: Loading relevant parameters from the configuration example

Server configuration with values applicable to work with `optical flow`: [server.ini](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/optical%20flow/server.ini).

To set the parameters manually, use the command `Server -> Edit config` from the top menu of the server.

#### Option 2. Reset all server configuration options

In order to configure the server configuration for positioning by optical flow, you can reset the configuration data to default values. To do this, delete the generated configuration file on the server:

* Execute the command `rm config/server.ini` from the directory with the location of the application `server.py`.
* Restart the server by selecting the command `Server -> Restart server` from the top menu item.

## Aruco

ArUco-Markers is a popular technology for positioning robotic systems using computer vision. Positioning is performed by obtaining information about the location of special visual markers.

This positioning system is flexible: markers can be placed on the floor, on the ceiling or on the walls. The main condition for positioning is to correctly enter coordinates in a special map of markers and mark its inclination relative to the floor.

If the marker map is located on the floor, there are some nuances: right after switching on the copter does not know its position until the camera sees the marker map. Therefore, some checks on the copter position and flight safety become irrelevant:

* It makes no sense to check the current position of the copter, as it is not defined immediately after loading, but the copter still has the take-off mechanism.
* It does not make sense to check the maximum distance of the animation start point to the current copter position, because it would never match at the start.
* The start action `fly` of the animation in the coordinate system `aruco_map` will have unpredictable consequences, because at the start the real position of the copter will be very different from the position calculated by the flight controller. Therefore, the only option for the parameter `start_action` in the [ANIMATION] section is `takeoff`: the ascent to the altitude defined by  `takeoff_height` the parameter from the [FLIGHT] section relative to the current position of the copter.

### clover ROS package configuration

Setting up the positioning system by ArUco markers is described in the `clover` [documentation](../aruco_map.md) .

Examples of `.launch` files for setting up a marker map on the floor:

* [clover.launch](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/aruco%20floor/launch/clover.launch)
* [aruco.launch](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/aruco%20floor/launch/aruco.launch)

### Flight controller configuration

Parameters adjusting the flight controller to allow flight by `ArUco`: [aruco.params](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/aruco%20floor/aruco.params).

To upload parameters to the selected copters in the table, use the command `Selected drones -> Send -> FCU parameters file` from the server top menu and specify the path to the flight controller parameters file.

### Client configuration

Configuration of the client with the values suitable to work with `ArUco`: [client.ini](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/aruco%20floor/client.ini).

To upload, use the command `Selected drones -> Send -> Configuration` from the server top menu. In the opened dialog box, select the option `Modify` and specify the path to the file `client.ini`.

### Server configuration

Configuration of the server with the values suitable to work with `ArUco`: [server.ini](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/aruco%20floor/server.ini).

To set the parameters manually, use the command `Server -> Edit config` from the server top menu.

## GPS

Satellite positioning is the preferred method of positioning for outdoor flights. GPS positioning requires [one of the officially supported modules for PX4](https://docs.px4.io/v1.9.0/en/gps_compass/). The accuracy of GPS positioning is about one meter. Using a ground correction station [GPS RTK](https://docs.px4.io/v1.9.0/en/gps_compass/rtk_gps.html) and special GPS RTK modules allows to achieve decimeter positioning accuracy.

### clover ROS package configuration

Setting up the positioning system by ArUco markers is described in the `clover` [documentation](../gps.md).

Example `.launch` file: [clover.launch](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/gps/launch/clover.launch).

### Flight controller configuration

Parameters adjusting the flight controller to allow flight by `gps`: [gps.params](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/gps/gps.params).

To upload parameters to the selected copters in the table, use the command `Selected drones -> Send -> FCU parameters file` from the server top menu and specify the path to the flight controller parameters file.

### Client configuration

Warning! The initial positioning point of the frame `gps` must be changed before starting the client. For example, the test flight zone point on the territory of Technopolis Moscow is selected.

Configuration of the client with the values suitable to work with `GPS`: [client.ini](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/gps/client.ini).

To upload, use the command `Selected drones -> Send -> Configuration` from the server top menu. In the opened dialog box, select the option `Modify` and specify the path to the file `client.ini`.

### Server configuration

Configuration of the server with the values suitable to work with `GPS`: [server.ini](https://github.com/CopterExpress/clever-show/blob/master/examples/positioning/gps/server.ini).

To set the parameters manually, use the command `Server -> Edit config` from the server top menu.
