# Работа с лазерным дальномером

> **Note** Документация для версий [образа](image.md), начиная с **0.18**. Для более ранних версий см. [документацию для версии **0.17**](https://github.com/CopterExpress/clever/blob/v0.17/docs/ru/laser.md).

## Дальномер VL53L1X

Рекомендуемая для Клевера модель дальномера – STM VL53L1X. Это дальномер может измерять расстояния от 0 до 4 м, при этом обеспечивая высокую точность измерений.

На [образе для Raspberry Pi](image.md) предустановлен соответствующий ROS-драйвер.

### Подключение к Raspberry Pi

> **Note** Для корректной работы лазерного дальномера с полетным контроллером необходима <a id="download-firmware" href="https://github.com/CopterExpress/Firmware/releases">кастомная прошивка PX4</a>. Подробнее про прошивку см. [соответствующую статью](firmware.md).

<script type="text/javascript">
    fetch('https://api.github.com/repos/CopterExpress/Firmware/releases').then(res => res.json()).then(function(data) {
        for (let release of data) {
            if (!release.prerelease && !release.draft && release.tag_name.includes('-clever.')) {
                document.querySelector('#download-firmware').href = release.html_url;
                return;
            }
        }
    });
</script>

Подключите дальномер по интерфейсу I²C к пинам 3V, GND, SCL и SDA:

<img src="../assets/raspberry-vl53l1x.png" alt="Подключение VL53L1X" height=600>

Если обозначенный пин GND занят, можно использовать другой свободный, используя [распиновку](https://pinout.xyz).

> **Hint** По интерфейсу I²C возможно подключать несколько периферийных устройств одновременно. Используйте для этого параллельное подключение.

### Включение

[Подключитесь по SSH](ssh.md) и отредактируйте файл `~/catkin_ws/src/clever/clever/launch/clever.launch` так, чтобы драйвер VL53L1X был включен:

```xml
<arg name="rangefinder_vl53l1x" default="true"/>
```

По умолчания драйвер дальномера передает данные в Pixhawk (через топик `/rangefinder/range`). Для просмотра данных из топика используйте команду:

```bash
rostopic echo /rangefinder/range
```

### Настройки PX4

Для использования данных с дальномера в [PX4 должен быть сконфигурирован](px4_parameters.md).

При использовании EKF2 (`SYS_MC_EST_GROUP` = `ekf2`):

* `EKF2_HGT_MODE` = `2` (Range sensor) – при полете над горизонтальным полом;
* `EKF2_RNG_AID` = `1` (Range aid enabled) – в остальных случаях.

При использовании LPE (`SYS_MC_EST_GROUP` = `local_position_estimator, attitude_estimator_q`):

* В параметре `LPE_FUSION` включен флажок "pub agl as lpos down" – при полете над горизонтальным полом.

### Получение данных из Python

Для получения данных из топика создайте подписчика:

```python
from sensor_msgs.msg import Range

# ...

def range_callback(msg):
    # Обработка новых данных с дальномера
    print 'Rangefinder distance:', msg.range

rospy.Subscriber('rangefinder/range', Range, range_callback)
```

Также существует возможность однократного получения данных с дальномера:

```python
from sensor_msgs.msg import Range

# ...

data = rospy.wait_for_message('rangefinder/range', Range)
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
