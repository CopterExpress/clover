# clever-show client

Application for remote synchronized drone control in show and emergency drone protection module.

* `client.py` - main communication and drone control module
* `failsafe.py` - drone protection module that creates a `/emergency_land` service  and controls the state of the drone according to the logic specified in `[FAILSAFE]` and `[EMERGENCY LAND]` sections of the `client.ini` configuration file .

## Manuals

* [Installation and startup](start-tutorial.md#client-installation-and-startup)
* [Client configuration](#client-configuration)

## Client workflow the Raspberry Pi image

The client is the `clever-show` service in the copter operating system. The service runs the [client.py](.../.../drone/client.py) script and starts automatically when the operating system boots. If it is necessary to apply the parameters of the updated client configuration, the service can be restarted. The `clever-show' service is designed to manage and configure the copter for group flight through the server application.

Along with the client, the emergency drone protection service `failsafe` is registered in the operating system. This service launches the script [failsafe.py](.../../drone/failsafe.py) and starts automatically when the operating system boots . The service can be restarted if it is necessary to apply parameters of the updated client configuration.

The ''failsafe'' service is designed for automatic landing of the copter in emergency situations:

* if there are no messages about the drone position from the `/mavros/vision_pose/pose` topic.
* in case of collision with objects in flight, when the distance between the current point where the copter is and the point where it should be according to the flight task exceeds the threshold value.

Also `failsafe` provides '`/emergency_land` ROS service, which can be accessed by the client upon command from the server for emergency drone landing.

Logs of both services are written to the`/var/log/syslog` file of the drone's on-board computer Raspberry Pi  operating system. The log files of the current session are available for viewing upon execution of the commands `journalctl -u clever-show` for the client and `journalctl -u failsafe` for the service of emergency drone protection in the terminal connected to Raspberry Pi. Logs may be useful for analysis of unexpected behavior or emergencies during the flight under control of the client application.

## Client configuration

### Configuration file

The client configuration is created according to the [specification](../../drone/config/spec/configspec_client.ini) and the default values for any parameter after the `default` keyword can be seen in it. All changes are saved in the configuration file `client.ini` in the folder `clever-show/drone/config`. Client configuration editing is available through the GUI module `Config editor` in the server application. To edit the configuration from the server, select the line with the client for which you want to change the configuration, click with the left mouse button on any cell in the line and select `Edit config` from the context menu.

The default configuration is fully functional and does not require changes to get the client up and running quickly.

In order to make a centralized upload of the configuration to all the copters the menu item `Send configurations` on [server](server.md#server section) should be used. It is allowed to load a partial configuration parameter file, with missing sections or default configuration parameters. There is also an option to upload a specific client configuration to dedicated copters: to do this, select the line with the client from which you want to copy the configuration to the dedicated clients in the server application, right-click on any cell from the line and select `Copy config to select` from the context menu.

### Parameter description

#### Main section

* `config_name` - configuration name
* `config_version` - configuration version
* `id` - the name of the copter displayed in the table. If the value is `/hostname` - name is determined from the `/etc/hostname` file.
* `clover_dir` - path to the directory with the [clover](https://github.com/CopterExpress/clover) ROS package. It is necessary to load files with aruco maps and launch configuration files for starting `clover` service. If the value `auto` - the client tries to define the required directory of the package `clover` (or `clever`) by itself through `rospkg` at the first launch. If the directory of the `clover` package cannot be determined, the value is set to `error` and files specific to ROS configuration of the `clover` package are not transferred from server to client.путь к директории с ROS пакетом [сlover](https://github.com/CopterExpress/clover).

#### SERVER section

This section contains the parameters of client-server network communication. Following parameters are available:

* `port` - TCP port to which incoming connections from the server will be accepted. When using the [use_broadcast](server.md#broadcast section) setting on the server, this port will be automatically configured by the client. *It is recommended to change the default value for security reasons (any five digits or more if another software does not use the selected port).*
* `host` - server IP address.
* `buffer_size` - buffer size for data transfer. *It is not recommended to modify. It is recommended to use the same value for the server and clients.*

#### BROADCAST section

This section turns on/off the broadcast packets receiving mechanism via UDP from the server.

* `use` -boolean value, determines whether to use or not the mechanism to receive broadcast packets via UDP from the server.
* `port` - UDP port for receiving broadcast messages.

#### TELEMETRY section

This section configures the telemetry stream to the server.

* `transmit` - boolean value, determines whether the telemetry data should be transmitted to the server.
* `frequency` - data transmission frequency, integer value, number of times per second.
* `log_resources` - boolean value, determines whether the state of the on-board computer: CPU and RAM load, processor temperature, temperature state, power system state will be logged in the client service log (only for Raspberry Pi).

#### FLIGHT section

This section contains settings affecting copter flight.

* `frame_id` - the name of the coordinate system, in relation to which the coordinates of the points for animation playback will be published. If value is `floor` - then the client publishes the static coordinate system with the name `floor` and the settings from [FLOOR_FRAME](#floor-frame-section). **Warning!** Make sure that the copter holds a position in this coordinate system. For this you can use the [Takeoff] (server.md# test commands) command from the server application. The copter will take off at an altitude of `takeoff_height` relative to the current one.
* `takeoff_height` - take-off height of the copter, in meters. Used at the beginning of animation playback or when testing the copter from the server.
* `takeoff_time` - maximum copter take-off time, in seconds.
* `reach_first_point_time` - maximum flight time to reach the first point of the animation, in seconds.
* `land_time` - hover time at the end point of the animation before landing, in seconds.
* `land_timeout` - landing timeout time, after which the copter motors are turned off, in seconds.

#### FLOOR FRAME section

This section describes the offset of the coordinate system with the name `floor` and is only used when you specify the parameter `frame_id` as `floor` in the [COPTERS](# partition-copters) section.

* `enabled` - boolean value, determines whether the frame `floor` should be published.
* `parent` - the name of the reference coordinate system relative to which the `floor` coordinate system will be located.
* `translation` - offset of the `floor` coordinate system along the axes (x, y, z) from the `parent` coordinate system, in meters.
* `rotation` - rotation of `floor` coordinate system by angles (roll, pitch, yaw) around axes (x, y, z) relative to `parent` coordinate system, in degrees.

**Warning!** Rotations `roll`, `pitch`, `yaw` are made sequentially in the stated order.

#### GPS FRAME section

This section describes creation of a coordinate system with the name `gps`. The initial position of this coordinate system is at a point with coordinates `lat` (latitude), `lon` (longitude). The angle of rotation of the coordinate system with respect to the north is set to `yaw` (in degrees, clockwise). The initial height of the coordinate system is set by the parameter `amsl`: it can be left equal to the initial height of the copter (`current`), or set to the altitude value in the [AMSL](https://en.wikipedia.org/wiki/Height_above_sea_level) format.

* `lat` - latitude in the WGS 84 system, in degrees
* `lon` - longitude in the WGS 84 system, in degrees
* `yaw` - angle of rotation of the coordinate system, in degrees
* `amsl` - height in meters

#### ANIMATION section

This section configures the animation processing. A separate module [animation](../../drone/modules/animation.py) is responsible for animation processing. When an animation is loaded on the copter, the module divides the sequence of animation frames into 5 key stages:

1. Copter is stationary at the beginning of the animation - `static_begin`.
2. Copter takes off - `takeoff'.
3. Copter follows the animation path - `route`.
4. Copter performs landing - `land`
5. Copter is stationary until the end of the animation file - `static_end`.

An animation frame is a set of data necessary to position the copter and determine its led strip color. In the current version of the software the animation frame is represented by a sequence of numbers `x y zaw r g b` in the line `.csv` of the animation file, where:

* `x', `y', `z' - copter coordinates in the current frame, in meters
* `yaw` - the copter's yaw in radians
* `r`, `g`, `b` - components of the color of the copter led strip, integers from 0 to 255

After splitting the animation into key stages, the module generates an output sequence of frames defining the position of the copter and its led strip color as well as the sequence of actions when flying to the first point of the animation. Adjustment of the module is performed using the following parameters:

* `start_action` - the first action when the animation playback starts. Available options:
  
  * `auto` - automatic choice of action between `takeoff` (take-off) or `fly` (instantaneous flight along points) based on the current level of altitude of the copter. If (`z` at the initial point of animation) > (take-off_level), the value is set to `takeoff`, otherwise the value is set to `fly`.
  * `fly` -  executing *the logic of immediate flight*
  * `takeoff` - execution of *the logic of flight to the first point*

If the copter takes off from the ground in the animation file, at the start of the animation, the instantaneous playback logic (fly)** will be applied: the copter with the motors turned off plays the color from the animation as long as it is stationary, turns the motors on before the moment of takeoff, then after `arming_time` starts to follow the points specified in the animation.

 If in the animation file the copter starts to fly in the air, at the start of the animation will be applied **flight logic to the first point (takeoff)**: The copter with the motors turned off plays the color from the animation as long as it is stationary, turns the motors on before takeoff, then takes off in `takeoff_height` time, then moves to the first point in `reach_first_point_time` and then starts to follow the points specified in the animation.

* `takeoff_level` - takeoff level to automatically detect the first action of the copter when the animation starts

* `ground_level` - ground level, used to check if the copter will try to fly underground when playing the animation. Available settings:
  
  * `current` - the current height level of the copter before the start is taken as the ground level.
  * z coordinate in meters
  
* `check_ground` - boolean value, determines whether to check the ground level in the animation.

* `frame_delay` - playback time of one frame in seconds

* `yaw` - copter rotation during flight to points, in degrees. If `nan', the copter preserves its original orientation in flight. If `animation` - the copter rotates by yaw from the animation file.

* `ratio` - scale of animation (ratio_x, ratio_y, ratio_z) along the axis (x, y, z)

* `common_offset` - Animation offset relative to the current system, common for all copters, in meters. List of 3 values (x, y, z): each value sets the offset on the corresponding axis.

* `private_offset` - Animation offset relative to the current system, only for this copter, in meters. List of 3 values (x, y, z): each value sets the offset on the corresponding axis.

* `[[OUTPUT]]` - flags that define which steps will be included in the output frame sequence.

#### LED section

Settings of the LED strip use on the copter. To configure the LED strip itself, you need to configure the `led.launch` file, see [LED strip in Clover](https://clover.coex.tech/ru/leds.html) for details.

* `use` - boolean value, determines whether the LED strip is used
* `takeoff_indication` - boolean value, determines whether to use the LEDs to indicate takeoff
* `land_indication` - boolean value, determines whether to use the LEDs to indicate land

#### FAILSAFE section

This section configures the program of emergency protection of the copter from a position loss or collision with an object. The `failsafe` module provides the service `/emergency_land` at startup, its configuration is located in the [EMERGENCY LAND](#emergency-land-section) section.

* `enabled` - boolean value, determines whether to use emergency protection in case of loss of visual position or collision with an object.
* `log_state` -  boolean value, determines whether the copter state will be logged in the service log:  `armed: {} | mode: {} | vis_dt: {:.2f} | pos_delta: {:.2f} | pos_dt: {:.2f} | range: {:.2f} | watchdog_action: {}`.
* `action` - action upon emergency protection triggering. Available options: `land` - landing of the copter in the flight controllers mode AUTO.LAND, `emergency_land` - landing of the copter with the gradual reduction of the motor power, `disarm` - switching off the motors. ** Attention!** It is not recommended to use the AUTO.LAND mode with the barometer turned off - when the altitude source in flight is lost, e.g. laser reading or visual position, the AUTO.LAND mode does not guarantee the landing of the copter, because it is oriented to the altitude reading. It is recommended to use the `emergency_land` mode to land the copter when positioning it using a visual position or laser and the possibility of losing data from these systems.
* `vision_pose_delay_after_arm` - time after takeoff of the copter in seconds, required to get the visual position. During this time after takeoff, the visual position loss protection will not work. This parameter is useful when using the emergency protection module in conjunction with the positioning system with aruco markers located on the floor: at takeoff copter has no visual position for some time.
* `vision_pose_timeout` - time in seconds after losing the visual position, after which the emergency protection is triggered.
* `position_delta_max` - the maximum distance between the current position and the point where the copter should now be in meters. Required to check for collision of the copter with objects. If the distance between the current position of the copter and the point where the copter should be now is greater than this number (in meters), an emergency protection is triggered.
* `disarm_timeout` - time after which the copter will unconditionally shut down the motors after the emergency protection is triggered, in seconds.

#### EMERGENCY LAND section

Settings of emergency landing parameters in case of `emergency_land` emergency protection action or when calling the `/emergency_land` ROS service.

* `thrust` - the initial power supplied to the motors in the event of a `emergency_land` action when the emergency protection is triggered. Measurementless value, from 0 (no thrust) to 1 (full thrust). For a guaranteed fit it is recommended to set it to a value 5-10 percent lower than the hanging gas (parameter `MPC_THR_HOVER` in px4). **Warning!** Incorrect configuration of this option may cause the copter to rise up instead of landing!
* `decrease_thrust_after` - the time after which the power on the motors slowly begins to decrease (in seconds) if the action `emergency_land` is selected when the emergency protection is triggered.

#### SYSTEM section

System settings for client's service commands

* `change_hostname` - boolean option, determines whether a change of hostname is required when renaming the copter `id`.
* `restart_after_rename` - boolean option, determines whether it is necessary to reboot the copter when renaming its `id` remotely from the server.

#### NTP section

In addition to time synchronization (with millisecond precision) using the chrony package, there is an alternative - the ability to use external (in case the local network has a connection to the Internet) or intranet NTP-servers. ** Attention!** For correct system operation, both the server and the clients** must use a single method of time synchronization (a set of parameters in this section). This section is fully unified for both server and clients.

* `use_ntp` - determines whether time synchronization using NTP will be used. (if `False', the local OS time will be used (synchronized automatically when using chrony). * It is recommended to use chrony instead of NTP*.
* `host` - host name or IP address of the NTP server (local or remote)
* `port` -  port used by the NTP server
