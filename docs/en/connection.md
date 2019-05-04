Connecting Pixhawk/Pixracer to Raspberry Pi
===

To program [autonomous flights](simple_offboard.md) [work with Pixhawk (Pixracer) over Wi-Fi](gcs_bridge.md), use [of a phone transmitter] (rc.md), and other functions, it is necessary to connect Raspberry Pi to Pixhawk (Pixracer).

Check operability of the connection [by running on Raspberry Pi](ssh.md):

```bash
rostopic echo /mavros/state
```

The `connected` field should contain value `True`.

Connection via USB
---

Connect Pixhawk/Pixracer to micro USB in Raspberry Pi with a USB cable.

Make sure that in Clever launch file (`~/catkin_ws/src/clever/clever/launch/clever.launch`), connection type is set to USB:

```xml
<arg name="fcu_conn" default="usb"/>
```

After the launch file is edited, restart package `clever`:

```bash
sudo systemctl restart clever
```

> **Hint** For correct operation of the Raspberry Pi connection to Pixhawk via USB, set value of [parameter](px4_parameters.md) `CBRK_USB_CHK` to 197848.

Connection via UART
---

TODO connection diagram

Make sure that in Clever launch file (`~/catkin_ws/src/clever/clever/clever.launch`), connection type is set to UART:

```xml
<arg name="fcu_conn" default="uart"/>
```

After the launch-file is edited, restart package `clever`:

```bash
sudo systemctl restart clever
```

> **Hint** For correct operation of the Raspberry Pi connection to Pixhawk via UART, set value of parameter`SYS_COMPANION` to 921600.

Connection to SITL
---

To connect locally/remotely to a running [SITL](sitl.md), set argument `fcu_conn` to `udp`, and `fcu_ip` to the IP address of the machine where SITL is running (`127.0.0.1` for local):

```xml
<arg name="fcu_conn" default="udp"/>
<arg name="fcu_ip" default="127.0.0.1"/>
```
