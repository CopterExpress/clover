# Работа с лазерным дальномером

## Дальномер VL53L1X

Рекомендуемая для Клевера модель дальномера – STM VL53L1X. Это дальномер может измерять расстояния от 0 до 4 м, при этом обеспечивая высокую точность измерений.

На [образе для Raspberry Pi](microsd_images.md) предустановлен соответствующий ROS-драйвер.

### Подключение к Raspberry Pi

Подключите дальномер по интерфейсу I²C к пинам 3V, GND, SCL и SDA:

<img src="../assets/raspberry-vl53l1x.png" alt="Подключение VL53L1X" height=600>

Если обозначенный пин GND занят, можно использовать другой свободный, используя [распиновку](https://pinout.xyz).

> **Hint** По интерфейсу I²C возможно подключать несколько периферийных устройств одновременно. Используйте для этого параллельное подключение.

### Включение

[Подключитесь по SSH](ssh.md) и отредактируйте файл `~/catkin_ws/src/clever/clever/launch/clever.launch` так, чтобы драйвер VL53L1X был включен:

```xml
<arg name="rangefinder_vl53l1x" default="true"/>
```

По умолчания драйвер дальномера передает данные в Pixhawk (через топик `/mavros/distance_sensor/rangefinder_sub`). Для просмотра данных из топика используйте команду:

```bash
rostopic echo mavros/distance_sensor/rangefinder_sub
```

### Настройки PX4

TODO

### Получение данных из Python

Для получения данных из топика создайте подписчика:

```python
from sensor_msgs.msg import Range

# ...

def range_callback(msg):
    # Обработка новых данных с дальномера
    print 'Rangefinder distance:', msg.range

rospy.Subscriber('mavros/distance_sensor/rangefinder_sub', Range, range_callback)
```

### Визуализация данных

Для построения графика по данным с дальномера может быть использован rqt_multiplot.

Для визуализации данных может быть использован rviz. Для этого необходимо добавить топик типа `sensor_msgs/Range` в визуализацию:

<img src="../assets/rviz-range.png" alt="Range в rviz">

См. [подробнее об rviz и rqt](rviz.md).

<!--
### Подключение к Pixhawk / Pixracer

Поддержка дальномера VL53L1X пока не реализована в прошивке PX4 (по состоянию на версию *1.8.2*).
-->
