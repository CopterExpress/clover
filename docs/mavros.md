MAVROS
===

Основная статья: http://wiki.ros.org/mavros

MAVROS (MAVLink + ROS) — это пакет для ROS, предоставляющий возможность управлять беспилотниками по протоколу MAVLink. MAVROS поддерживает полетные стеки PX4 и APM. Связь организовывается по UART, USB, TCP или UDP.

MAVROS подписывается определенные ROS-топики в ожидании команд, публикует в другие топики телеметрию, и предоставляет сервисы.

MAVROS, как и другие ROS-пакеты, запускается с помощью утилиты ``roslaunch``. Запуск для соединения с PixHawk (PX4) по UART на Raspberry Pi:

```
roslaunch mavros px4.launch fcu_url:=/dev/ttyAMA0:921600
```

Соединение к [PX4 SITL](sitl.md):
```
roslaunch mavros px4.launch fcu_url:=udp://@<ip>:14557
```

Где ``<ip>`` — это ip-адрес машины, на которой запущен PX4. Если он запущен локально, то ``<ip>``=127.0.0.1.

Основные сервисы
---

```/mavros/set_mode``` — установить [полетный режим](modes.md) контроллера. Обычно устанавливается режим OFFBOARD (для управления с Raspberry Pi).

```/mavros/cmd/arming``` — включить или выключить моторы беспилотника (изменить armed-статус).

Основные публикуемые топики
---

```/mavros/state``` — статус подключения к полетному контроллеру. Режим полетного контроллера.

```/mavros/local_position/pose``` — локальная позиция коптера в системе координат ENU.

```/mavros/local_position/velocity``` — текущая скорость в локальных координатах. Угловые скорости.

```/mavros/global_position/global``` — текущая глобальная позиция (широта, долгота, высота).

```/mavros/global_position/local``` — глобальная позиция в системе координат [UTM](https://ru.wikipedia.org/wiki/Система_координат_UTM).

```/mavros/global_position/rel_alt``` — относительная высота (относительно высоты включения моторов).

Основные топики для публикации
---

```/mavros/setpoint_position/local``` — установить целевую позицию  и рысканье (yaw) беспилотника (в системе координат ENU).

```/mavros/setpoint_velocity/cmd_vel``` — установить целевую линейную скорость беспилотника.

```/mavros/setpoint_attitude/attitude``` и ```/mavros/setpoint_attitude/att_throttle``` — установить целевую ориентацию (Attitude) и уровень газа.

```/mavros/setpoint_attitude/cmd_vel``` и ```/mavros/setpoint_attitude/att_throttle``` — установить целевые угловые скорости и уровень газа.

### Топики для посылки raw-пакетов:

```/mavros/setpoint_raw/local``` — отправка пакета [SET_POSITION_TARGET_LOCAL_NED](https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_LOCAL_NED). Позволяет установить целевую позицию/целевую скорость и целевое рысканье/угловую скорость по рысканью. Выбор устанавливаемых величин осуществляется с помощью поля ``type_mask``.

```/mavros/setpoint_raw/attitude``` — отправка пакета [SET_ATTITUDE_TARGET](https://pixhawk.ethz.ch/mavlink/#SET_ATTITUDE_TARGET). Позвлояет установить целевую ориенатацию /угловые скорости и уровень газа. Выбор устанавливаемых величин осуществляется с помощью поля ``type_mask``

```/mavros/setpoint_raw/global``` — отправка пакета [SET_POSITION_TARGET_GLOBAL_INT](https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT). Позволяет установить целевую позицию в глобальных координатах (ширина, долгота, высота), а также скорости полета. **Не поддерживается в PX4** (issue](https://github.com/PX4/Firmware/issues/7552)).
