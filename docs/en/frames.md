Coordinate systems (frames)
===

> **Note** Documentation for the [image](microsd_images.md), versions, starting with **0.15**. For older versions refer to [documentation for version **0.14**](https://github.com/CopterExpress/clever/blob/v0.14/docs/ru/frames.md).

![Clever coordinates systems (TF2)](../assets/frames.png)

Main frames in package `clever`:

* `map` coordinates relative to the point of flight controller initialization: the white grid in the illustration;
* `base_link` — coordinates relative to the quadcopter: schematic image of the quadcopter in the illustration;
* `body` — coordinates relative to the quadcopter regardless of pitch and roll: red, blue and green lines in the illustration.

> **Hint** In accordance with [the agreement](http://www.ros.org/reps/rep-0103.html), for frames associated with the copter, the X-axis directed forward, Y – to the left, and Z – up.

More clearly, 3D visualization of the coordinate systems can be viewed using [rviz](rviz.md).