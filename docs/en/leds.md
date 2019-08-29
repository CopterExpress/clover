# Working with a LED strip

> **Note** The following applies to image version 0.18 and up. See [previous version of the article](leds_old.md) for older images.

Clever drone kits contain addressable LED strips based on *ws281x* drivers. Each LED may be set to any one of 16 million possible colors (each color is encoded by a 24-bit number). This allows making the Clever flight more spectacular, as well as show flight modes, display stages of current user program, and notify the pilot of other events.

<img src="../assets/clever-led.png" class="center" width=600>

Our [Raspberry Pi image](image.md) contains preinstalled modules for interfacing with the LED strip. They allow the user to:

* manage LED strip effects and animations (high-level control);
* control individual LED colors (low-level control);
* configure the strip to display flight events.

> **Caution** LED strip can consume a lot of power! Powering it from a Raspyerry Pi may overload the computer's power circuitry. Consider using a separate BEC as a power source.

## High-level control

1. Connect the +5v and GND leads of your LED to a power source and the DIN (data in) lead to GPIO21. Consult the [assembly instructions](assemble_4.md#Connecting-the-LED-strip-to-Raspberry-Pi) for details.
2. Enable LED strip support in `~/catkin_ws/src/clever/clever/launch/clever.launch`:

    ```xml
    <arg name="led" default="true"/>
    ```

3. Configure the *ws281x* parameters in `~/catkin_ws/src/clever/clever/launch/led.launch`. Change the number of addressable LEDs and the GPIO pin used for control to match your configuration:

    ```xml
    <param name="led_count" value="30"/>  <!-- Number of LEDs in the strip -->
    <param name="gpio_pin" value="21"/>   <!-- GPIO data pin -->
    ```

High-level interface allows changing current effect (or animation) on the strip. It is exposed as the `/led/set_effect` service. It has the following arguments:

* `effect` is the name of requested effect.
* `r`, `g`, `b` are [RGB](https://en.wikipedia.org/wiki/RGB) components of effect color. Each component is an integer in a 0 to 255 range.

Currently available effects are:

* `fill` (or an empty string) fills the whole strip with the requested color;
* `blink` turns the strip on and off, setting it to the requested color;
* `blink_fast` is the same, but faster;
* `fade` fades smoothly to the requested color;
* `wipe` fills the strip with the requested color one LED at a time;
* `flash` blinks twice and returns to the previous effect;
* `rainbow` creates a rainbow-like shifting effect;
* `rainbow_fill` cycles the strip through rainbow colors, filling the whole strip with the same color.

Python example:

```python
import rospy
from clever.srv import SetLEDEffect

# ...

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

# ..

set_effect(r=255, g=0, b=0)  # fill strip with red color
rospy.sleep(2)

set_effect(r=0, g=100, b=0)  # fill strip with green color
rospy.sleep(2)

set_effect(effect='fade', r=0, g=0, b=255)  # fade to blue color
rospy.sleep(2)

set_effect(effect='flash', r=255, g=0, b=0)  # flash twice with red color
rospy.sleep(5)

set_effect(effect='blink', r=255, g=255, b=255)  # blink with white color
rospy.sleep(5)

set_effect(effect='rainbow')  # show rainbow
```

You can also set colors from your Bash shell:

```bash
rosservice call /led/set_effect "{effect: 'fade', r: 0, g: 0, b: 255}"
```

```bash
rosservice call /led/set_effect "{effect: 'rainbow'}"
```

## Configuring event visualizations

It is possible to display current flight controller status and notify the user about some events with the LED strip. This is configured in the `~/catkin_ws/src/clever/clever/launch/led.launch` file in the *events effects table* section. Here is a sample configuration:

```xml
startup: { r: 255, g: 255, b: 255 }
connected: { effect: rainbow }
disconnected: { effect: blink, r: 255, g: 50, b: 50 }
<!-- ... -->
```

The left part is one of the possible events that the strip reacts to. The right part contains the effect description that you want to execute for this event. Here is the list of supported events:

* `startup` – Clever system startup;
* `connected` – successful connection to the flight controller;
* `disconnected` – connection to the flight controller lost;
* `armed` – flight controller transitioned to armed state;
* `disarmed` – flight controller transitioned to disarmed state;
* `stabilized`, `acro`, `rattitude`, `altctl`, `posctl`, `offboard`, `mission`, `rtl`, `land` – transition to said flight mode;
* `error` – an error occured in one of ROS nodes or in the flight controller (*ERROR* message in `/rosout`);
* `low_battery` – low battery (threshold is set in the `threshold` parameter).

> **Note** You need to [calibrate the power sensor](power.md#calibrating-the-power-sensor) for the `low_battery` event to work properly.

In order to disable LED strip notifications set `led_notify` argument in `~/catkin_ws/src/clever/clever/launch/led.launch` to `false`:

```xml
<arg name="led_notify" default="false"/>
```

## Low-level control

You can use the `/led/set_leds` ROS service to control individual LEDs. It accepts an array of LED indices and desired colors.

Python example:

```python
import rospy
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState

# ...

set_leds = rospy.ServiceProxy('led/set_leds', SetLEDs)  # define proxy to ROS service

# ...

# switch LEDs number 0, 1 and 2 to red, green and blue color:
set_leds([LEDState(0, 255, 0, 0), LEDState(1, 0, 255, 0), LEDState(2, 0, 0, 255)])
```

You can also use this service from the your Bash shell:

```bash
rosservice call /led/set_leds "leds:
- index: 0
  r: 50
  g: 100
  b: 200"
```

Current LED strip state is published in the `/led/state` ROS topic. You can view the contents of this topic from your Bash shell:

```bash
rostopic echo /led/state
```
