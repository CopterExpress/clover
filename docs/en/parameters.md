# PX4 Parameters

Full documentation on PX4 parameters: https://docs.px4.io/master/en/advanced_config/parameter_reference.html.

For changing PX4 parameters, use QGroundControl software, [connect to Clover over Wi-Fi](gcs_bridge.md) or USB. Go to *Vehicle Setup* panel (click on the QGroundControl logo in the top-left corner) and choose *Parameters* menu.

## Recommended values

### Common parameters

|Parameter|Value|Comment|
|-|-|-|
|`SENS_FLOW_ROT`|0 (*No rotation*)|If using *PX4Flow* hardware, keep the default value|
|`SENS_FLOW_MINHGT`|0.0|For [VL53L1X](laser.md) rangefinder|
|`SENS_FLOW_MAXHGT`|4.0|For [VL53L1X](laser.md) rangefinder|
|`SENS_FLOW_MAXR`|10.0||
|`SYS_HAS_MAG`|0|If impossible to run the magnetometer (*No mags found* error)|

### Estimator subsystem parameters

In case of using LPE ([COEX patched firmware](firmware.md)):

|Parameter|Value|Comment|
|-|-|-|
|`LPE_FUSION`|86|Checkboxes: *flow* + *vis* + *land detector* + *gyro comp*. If flying over horizontal floor *pub agl as lpos down* checkbox is allowed.<br>Details: [Optical Flow](optical_flow.md), [ArUco markers](aruco_map.md), [GPS](gps.md).|
|`LPE_VIS_DELAY`|0.0||
|`LPE_VIS_Z`|0.1||
|`LPE_FLW_SCALE`|1.0||
|`LPE_FLW_R`|0.2||
|`LPE_FLW_RR`|0.0||
|`LPE_FLW_QMIN`|10||
|`ATT_W_EXT_HDG`|0.5|Enabling usage of external yaw angle (when navigating using [markers map](aruco_map.md))|
|`ATT_EXT_HDG_M`|1 (*Vision*)||
|`ATT_W_MAG`|0|Disabling usage of the magnetometer (when navigating indoor)|

In case of using EKF2 (official firmware):

<!-- markdownlint-disable MD044 -->

|Parameter|Value|Comment|
|-|-|-|
|`EKF2_AID_MASK`\*|26|Checkboxes: *flow* + *vision position* + *vision yaw*.<br>Details: [Optical Flow](optical_flow.md), [ArUco markers](aruco_map.md), [GPS](gps.md).|
|`EKF2_OF_DELAY`|0||
|`EKF2_OF_QMIN`|10||
|`EKF2_OF_N_MIN`|0.05||
|`EKF2_OF_N_MAX`|0.2||
|`EKF2_HGT_MODE`\*|3 (*Vision*)|If the [rangefinder](laser.md) is present and flying over horizontal floor – 2 (*Range sensor*)|
|`EKF2_EVA_NOISE`|0.1 rad or 5 deg||
|`EKF2_EVP_NOISE`|0.1||
|`EKF2_EV_DELAY`|0||
|`EKF2_MAG_TYPE`|5 (*None*)|Disabling usage of the magnetometer (when navigating indoor)|

\* — starting from PX4 version 1.14, the parameters marked with an asterisk are replaced with the following:

|Parameter|Value|Comment|
|-|-|-|
|`EKF2_EV_CTRL`|11|Checkboxes: *Horizontal position* + *Vertical position* + *Yaw*|
|`EKF2_GPS_CTRL`|0|All checkboxes are disabled|
|`EKF2_BARO_CTRL`|0 (*Disabled*)|Barometer is disabled|
|`EKF2_OF_CTRL`|1 (*Enabled*)|Optical flow is enabled|
|`EKF2_HGT_REF`|3 (*Vision*)|If the [rangefinder](laser.md) is present and flying over horizontal floor – 2 (*Range sensor*)|
|`EKF2_RNG_CTRL`|2 (*Enabled*)|Range sensor is enabled|

<!-- markdownlint-enable MD031 -->

> **Info** See also: list of default parameters of the [Clover simulator](simulation.md): https://github.com/CopterExpress/clover/blob/master/clover_simulation/airframes/4500_clover.

## Additional information

The `SYS_MC_EST_GROUP` parameter defines the estimator subsystem to use.

Estimator subsystem is a group of modules that calculates the current state of the copter using readings from the sensors. The copter state includes:

* Angle rate of the copter – roll_rate, pitch_rate, yaw_rate;
* Copter orientation (in the local coordinate system) – roll, pitch, yaw (one of presentations);
* Copter position (in the local coordinate system) – x, y, z;
* Copter speed (in the local coordinate system) – vx, vy, vz;
* Global coordinates of the copter – latitude, longitude, altitude;
* Altitude above the surface;
* Other parameters (the drift of gyroscopes, wind speed, etc.).

`SYS_AUTOCONFIG` — resets all parameters (sets to 1).

## EKF2

`EKF2_AID_MASK` — selects sensors that are used by EKF2 to calculate the copter state.

`EKF2_HGT_MODE` is the main source of height data (z in the local coordinate system):

* 0 – pressure reading on the barometer.
* 1 – GPS.
* 2 – distance meter (for example, vl53l1x).
* 3 – data from VPE.

Variant 2 is the most accurate; however, it is correct to use it only if the surface the copter flies over is flat. Otherwise, the Z axis origin will move up and down with the altitude of the surface.

## Multicopter Position Control

These parameters adjust the flight of the copter by position (POSCTL, OFFBOARD, AUTO modes).

`MPC_THR_HOVER` — hovering throttle. This option is to set to the approximate percentage of throttle needed to make the copter maintain its altitude. If copter has a tendency to gain or lose altitude during the hovering mode, reduce or increase this value.

`MPC_XY_P` – position factor *P* of the ESC. This parameter affects how sharply the copter will react to the position commands. A too high value may cause overshoots.

`MPC_XY_VEL_P` – speed factor *P* of the ESC. This parameter also affects the accuracy and sharpness of copter execution of the given position. A too high value may cause overshoots.

`MPC_XY_VEL_MAX` — the maximum horizontal speed in POSCTL, OFFBOARD, AUTO modes.

`MPC_Z_P`, `MPC_Z_VEL_P` – vertical position and speed factors *P* of the ESCs they determine the copter's ability to maintain the desired altitude.

`MPC_LAND_SPEED` is the vertical velocity of landing in the LAND mode.

## LPE + Q attitude estimator

These parameters configure the behavior of the `lpe` and `q` modules, which compute the state (orientation, position) of the copter. These parameters apply **only** if the `SYS_MC_EST_GROUP` parameter is set to `1` (local_position_estimator, attitude_estimator_q).

## Commander

Prearm checks, switching the modes and states of the copter.

## Sensors

Enabling, disabling and configuring various sensors.
