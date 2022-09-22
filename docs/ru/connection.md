# Подключение Raspberry Pi к полетному контроллеру

Для программирования [автономных полетов](simple_offboard.md), [работы с Pixhawk (Pixracer) по Wi-Fi](gcs_bridge.md), использования [телефонного пульта](rc.md) и других функций необходимо соединение Raspberry Pi и полетного контроллера.

## Подключение по USB

Основным способом подключения является подключение по интерфейсу USB.

1. Соедините Raspberry Pi и полетный контроллер micro-USB to USB кабелем.
2. [Подключитесь в Raspberry Pi по SSH](ssh.md).
3. Убедитесь в работоспособности подключения, [выполнив на Raspberry Pi](ssh.md):

    ```bash
    rostopic echo /mavros/state
    ```

    Поле `connected` должно содержать значение `True`.

> **Hint** Для корректной работы подключения Raspberry Pi и Pixhawk по USB необходимо установить значение [параметра](parameters.md) `CBRK_USB_CHK` на 197848.

## Подключение по UART

<!-- TODO схема подключения -->

Дополнительным способом подключения является подключение подключение по интерфейсу UART.

1. Подключите Raspberry Pi к полетному контроллеру по UART. Для этого соедините кабелем порт TELEM 2 на полетном контроллере к пинам на Raspberry Pi следующем образом: черный провод (GND) к Ground, зеленый (*UART_RX*) к *GPIO14*, желтый (*UART_TX*) к *GPIO15*. Красный провод (*5V*) подключать не нужно.
2. Измените значения параметров PX4: `MAV_1_CONFIG` на TELEM 2, `SER_TEL2_BAUND` на 921600 8N1. В PX4 до версии v1.10.0 необходима установка параметра `SYS_COMPANION` в значение 921600.
3. [Подключитесь в Raspberry Pi по SSH](ssh.md).
4. Поменяйте в launch-файле Клевера (`~/catkin_ws/src/clover/clover/launch/clover.launch`) тип подключения на UART:

    ```xml
    <arg name="fcu_conn" default="uart"/>
    ```

    При изменении launch-файла необходимо перезапустить сервис `clover`:

    ```bash
    sudo systemctl restart clover
    ```

**Далее**: [Подключение QGroundControl по Wi-Fi](gcs_bridge.md).
