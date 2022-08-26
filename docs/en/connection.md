# Connecting Raspberry Pi to the flight controller

In order to program [autonomous flights](simple_offboard.md), [work with Pixhawk or Pixracer over Wi-Fi](gcs_bridge.md), use [controller app](rc.md) and access other functions you need to connect your Raspberry Pi to the flight controller.

## USB connection

USB connection is the preferred way to connect to the flight controller.

1. Connect your FCU to the Raspberry Pi using a microUSB to USB cable.
2. [Connect to the Raspberry Pi over SSH](ssh.md).
3. Make sure the connection is working by [running the following command on the Raspberry Pi](ssh.md):

    ```bash
    rostopic echo /mavros/state
    ```

    The `connected` field should have the `True` value.s

> **Hint** You need to set the `CBRK_USB_CHK` [parameter](px4_parameters.md) to 197848 for the USB connection to work.

## UART connection

> **Note** In the image version **0.20** `clever` package and service was renamed to `clover`. See [previous version of the article](https://github.com/CopterExpress/clover/blob/v0.19/docs/en/connection.md) for older images.

<!-- TODO: Connection scheme -->

UART connection is another way for the Raspberry Pi and FCU to communicate.

1. Connect the TELEM 2 port on the flight controller using a UART cable to the Raspberry Pi pins following this instruction: the black cable (*GND*) to Ground, the green cable (*UART_RX*) to *GPIO14*, the yellow cable (*UART_TX*) to *GPIO15*. Do not connect the red cable (*5V*).
2. Set the PX4 parameters: `MAV_1_CONFIG` to TELEM 2, `SER_TEL2_BAUND` to 921600 8N1. In PX4 of version prior to v1.10.0 the parameter `SYS_COMPANION` should be set to 921600.
3. [Connect to the Raspberry Pi over SSH](ssh.md).
4. Change the connection type in `~/catkin_ws/src/clover/clover/launch/clover.launch` to UART:

    ```xml
    <arg name="fcu_conn" default="uart"/>
    ```

    Be sure to restart the `clover` service after editing the .launch file:

    ```bash
    sudo systemctl restart clover
    ```

**Next**: [Using QGroundControl over Wi-Fi](gcs_bridge.md)
