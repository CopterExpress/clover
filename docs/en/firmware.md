Pixhawk / Pixracer firmware flashing
===

Pixhawk or Pixracer firmware may be flashed using QGroundControl or command line utilities.

Various releases of stable PX4 firmwares are available from [Releases at GitHub] (https://github.com/PX4/Firmware/releases).

The name of the firmware file contains encoded information about the target circuit-board and the release. Examples:

* `px4fmu-v2_default.px4` — firmware for Pixhawk with EKF2.
* `px4fmu-v2_lpe.px4` — firmware for Pixhawk with LPE.
* `px4fmu-v4_default.px4` — firmware for Pixhawk with EKF2 and LPE (*Clever 3*).
* `px4fmu-v3_default.px4` — firmware for newer Pixhawk versions (rev. 3 chip, see Fig. + Bootloader v5) with EKF2 and LPE.

![STM revision](../assets/stmrev.jpg)

> **Note** To download the `px4fmu-v3_default.px4` file, you may need to use the `force_upload` command in the command prompt.

QGroundControl
---

In QGroundControl, go to Firmware. **After** that, connect Pixhawk / Pixracer via USB.

Select PX4 Flight Stack. To download and upload the standard firmware (the version with EKF2 for Pixhawk), select the "Standard Version" menu item, to load your own firmware file, select "Custom firmware file...", then click OK.

> **Warning** Do not disconnect the USB cable until the flashing process is complete.

TODO: Figure.

Command prompt
---

PX4 may be compiled from the source and automatically loaded to the circuit-board from the command prompt.

To do this, clone the PX4 repository:

```bash
git clone https://github.com/PX4/Firmware.git
```

Select the appropriate version (tag) using `git checkout`. Then compile and upload the firmware:

```
make px4fmu-v4_default upload
```

Where `px4fmu-v4_default` is the required firmware version.

To upload the `v3` firmware to Pixhawk, you may need the `force_upload` command:

```
make px4fmu-v3_default force-upload
```