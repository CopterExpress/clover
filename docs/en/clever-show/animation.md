# Animation module

A separate module [animation](../../drone/modules/animation.py) is responsible for animation processing. When animation is loaded on the copter, the module divides the sequence of animation frames into 5 key stages:

1. The copter is stationary at the beginning of the animation - `static_begin`.
2. Copter takes off - `takeoff`.
3. The copter follows the route of animation - `route`.
4. Copter performs landing - `land`.
5. The copter is stationary until the animation file - `static_end` is finished.

An animation frame is a set of data needed to position the copter and determine its LED strip color. In the current version of the software the animation frame is represented by a sequence of numbers `x y zaw r g b` in the lines of the `.csv` animation file where:

* `x`, `y`, `z` - copter coordinates in the current frame, in meters
* `yaw` - copter yaw, in radians
* `r`, `g`, `b` - сopter LED strip color components, integers from 0 to 255

After splitting the animation into key stages, the module generates an output sequence of frames defining the position of the copter and its lED strip color as well as the sequence of actions during the flight to the first point of the animation.

You can configure the module in the [ANIMATION](client.md#раздел-animation) section.

Preliminary selection of frames is carried out using a set of [[OUTPUT]] flags, which set which frame sequences out of 5 key stages will be used in flight and which will not.

The key parameter that defines the logic of animation playback is the `start_action` parameter that defines the first action when the animation playback starts. Available options of its values:

* `auto` - automatic choice of action between `takeoff` (take-off) or `fly` (instantaneous flight along points) based on the current level of altitude of the copter. If (`z` at the initial point of animation) > (take-off_level), the value is set to `takeoff`, otherwise the value is set to `fly`.
* `fly` - executing *the logic of immediate flight*
* `takeoff` - execution of *the logic of flight to the first point*

If the copter takes off from the ground in the animation file, at the start of the animation, the instantaneous playback logic (fly)** will be applied: the copter with the motors turned off plays the color from the animation as long as it is stationary, turns the motors on before the moment of takeoff, then after `arming_time` starts to follow the points specified in the animation.

 If in the animation file the copter starts to fly in the air, at the start of the animation will be applied **flight logic to the first point (takeoff)**: The copter with the motors turned off plays the color from the animation as long as it is stationary, turns the motors on before takeoff, then takes off in `takeoff_height` time, then moves to the first point in `reach_first_point_time` and then starts to follow the points specified in the animation.

If the `start_action` parameter is set to `takeoff` and the `takeoff` flag in the [[OUTPUT]] section is set to `True`, the sequence of frames with the copter taking off by points is replaced by 2 consecutive actions:

* takeoff relative to the current position to the height of `takeoff_height` for `takeoff_time`
* flying in a straight line to the starting point of the copter route frame sequence (`route`) during `reach_first_point_time`.

## Animation preparation and upload

Create object animations in [Blender](https://www.blender.org) or use [examples](.../../examples/animations).

The copter can be represented by any three-dimensional object (e.g., a cube or a ball), and the LED strip color will be extracted from the object color property. Consider the following facts and recommendations when creating an animation:

* For easy conversion and uploading of animation to copters, objects corresponding to copters should have names corresponding to the names of these copters.
* Blender distance units are converted into meters
* The default delay between frames in [Copter settings](.../../drone/config/spec/configspec_client.ini) is 0.1 seconds (parameter `frame_delay` in ANIMATION section), pay attention when setting the frame rate in Blender animation.
* Make sure that the speed of the copters is not too high ( maximum of 3 m/s for the room, maximum of 5 m/s for the street): the addon will give a warning but will still convert the animation.

Convert the animation with [Blender addon](blender-addon.md).

If there are several objects in the animation and their names match the names of the copters, load the animation folder on the selected copters in the table using the [server](server.md#selected-drones section) command `Send -> Animations`.

Also any animation file can be loaded separately on all selected copters in the table with the command `Send -> Animation`.

## Animation analysis

If you need information about which points the copter will fly to as a result of loading the animation on the current client parameters, use the [animation_info](../../tools/animation_info.py) utility .

```cmd
usage: python animation_info.py [-h] [--config] [animation]

Get animation info

positional arguments:
  animation   Path to animation. Default is
              ../examples/animations/basic/basic.csv.

optional arguments:
  -h, --help  show this help message and exit
  --config    Set this option to print config info.
```

This utility displays full information about the client's animation and configuration settings (optional), as well as both possible playback options for the animation, which allows to analyze the actions of the copters before a real flight.
