Simple offboard
===

Модуль `simple_offboard` пакета `clever` предназначен для упрощенного программирования автономного дрона (режим `OFFBOARD`). Модуль автомтически трансформирует систему координат. 

Основные функции – `get_telemetry` (получение всей телеметрии скопом), `navigate` (полет в заданную точку по прямой).

[Подробнее о системах координат](/docs/frames.md).

Общие для сервисов параметры:

* `auto_arm` = `true`/`false` – перевести коптер в OFFBOARD и заармить автоматически (**коптер взлетит, если находится на полу!**)
* `frame_id` — система координат в TF2, в которой заданы координаты и рысканье (yaw), [описание систем координат](/docs/frames.md);
* `update_frame` — считать ли систему координат изменяющейся (например, `false` для `local_origin`, `fcu`, `fcu_horiz`, `true` для `marker_map`);
* `x`, `y` – горизонтальные координаты в системе координат `frame_id`;
* `z` — высота в системе координат `frame_id`;
* `yaw` — рысканье в радианах в системе координат `frame_id` (0 – коптер смотрит по оси X);
* `yaw_rate` — угловая скорость по рысканью в радианах в секунду (против часовой);
* `thrust` — уровень газа (от 0 [нет газа] до 1 [полный газ]).

Использование из языка Python
---

Объявление прокси ко всем сервисам:

```python
import rospy
from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('foo')

# Создаем прокси ко всем сервисам:

navigate = rospy.ServiceProxy('/navigate', srv.Navigate)

set_position = rospy.ServiceProxy('/set_position', srv.SetPosition)
set_position_yaw_rate = 
rospy.ServiceProxy('/set_position/yaw_rate', srv.SetPositionYawRate)

set_position_global = rospy.ServiceProxy('/set_position_global', srv.SetPositionGlobal)
set_position_global_yaw_rate = rospy.ServiceProxy('/set_position_global/yaw_rate', srv.SetPositionGlobalYawRate)

set_velocity = rospy.ServiceProxy('/set_velocity', srv.SetVelocity)
set_velocity_yaw_rate = rospy.ServiceProxy('/set_Velocity/yaw_rate', srv.SetVelocityYawRate)

set_attitude = rospy.ServiceProxy('/set_attitude', srv.SetAttitude)
set_attitude_yaw_rate = rospy.ServiceProxy('/set_attitude/yaw_rate', srv.SetAttitudeYawRate)

set_rates_yaw = rospy.ServiceProxy('/set_rates/yaw', srv.SetRatesYaw)
set_rates = rospy.ServiceProxy('/set_rates', srv.SetRates)

get_telemetry = rospy.ServiceProxy('/get_telemetry', srv.get_telemetry)
release = rospy.ServiceProxy('/release', Trigger)
```

Неиспользуемые фукнции-прокси можно удалить из кода.

Список сервисов
---

### get_telemetry

Получить полную телеметрию коптеру. Параметр: `frame_id` – фрейм для значений `x`, `y`, `z`, `vx`, `vy`, `vz`. Пример: `local_origin`, `fcu_horiz`, `aruco_map`.

Ответ:

* `frame_id` – фрейм
* `connected` – есть ли подключение к FCU
* `armed` – состояние `armed` винтов (винты включены, если true)
* `mode` - текущий [полетный режим](/docs/modes.md)
* `x, y, z` – локальная позиция коптера
* `lat, lon` – широта, долгота (при наличии [gps](/docs/gps.md))
* `vx, vy, vz` – скорость коптера
* `pitch` – угол по тангажу (радианы)
* `roll` – угол по крену (радианы)
* `yaw` – угол по рысканью в фрейме `frame_id`
* `pitch_rate` – угловая скорость по тангажу
* `roll_rate` – угловая скорость по крену
* `yaw_rate` – угловая скорость по рысканью
* `voltage` – общее напряжение аккумулятор
* `cell_voltage` – напряжение аккумулятора на ячейку

Пример. Вывести координаты x, y и z коптера в локальной системе координат:

```python
telemetry = get_telemetry()
print telemetry.x, telemetry.y, telemetry.z
```

### navigate

Прилететь в обозначенную точку по прямой.

Параметры:

* x, y, z – координаты в системе `frame_id`
* yaw – угол по рысканью
* speed – скорость полета (скорость движения setpoint)
* frame_id, update_frame, auto_arm.

Примеры:

```python
# плавно взлететь на высоту 1.5 м со скоростью взлета 0.5 м/с
navigate(0, 0, 1.5, speed=0.5, frame_id='fcu_horiz', auto_arm=True)
```

```python
# прилететь по прямой в точку 5:0 (высота 2)
# в локальной системе координат со скоростью 0.8 м/с
navigate(5, 0, 3, speed=0.8)
```


```python
# пролететь вправо относительно коптера на 3 м
navigate(0, -1, 0, speed=1, frame_id='fcu_horiz')
```

```python
# прилететь в точку 3:2 (высота 2) в системе координат маркерного поля
# со скоростью 1 м/с
navigate(3, 2, 2, speed=1, frame_id='aruco_map', update_frame=True)
```

### set_position

Установить цель по позиции и рысканью. Для полета на точку по прямой или взлета используйте более высокоуровневую функицю `navigate`.

Параметры: x, y, z, yaw, frame_id, update_frame

Задание позиции относительно коптера:

```python
set_position(x=0, y=0, z=3, frame_id='fcu_horiz', auto_arm=true)  #  взлет на 3 метра
```

```python
set_position(x=1, y=0, z=0, frame_id='fcu_horiz')  # пролететь вперед на 1 метр
```

```python
set_position(x=0, y=-1, z=0, frame_id='fcu_horiz')  # пролететь вправо на 1 метр
```

Задание позиции относительно системы маркеров
(фрейм `aruco_map` не будет опубликован, пока коптер хоть раз не увидит один из маркеров):

```python
set_position(x=2, y=2, z=3, frame_id='aruco_map', update_frame=True)  #  полет в координату 2:2, высота 3 метра
```

### set_position_yaw_rate

Установить позицию и угловую скорость по рысканью.

Параметры: x, y, z, yaw_rate, frame_id, update_frame

### set_position_global

Полет в позицию в глобальной системе координат (широта/долгота).

Параметры: lat (широта), lon (долгота), z (высота в системе координат frame_id), yaw (рысканье в системе координат frame_id), update_frame.

Полет в глобальную точку (оставаясь на текущей высоте):
```python
set_position_global(lat=55.707033, lon=37.725010, z=0, frame_id='fcu_horiz')
```

### set_position_global_yaw_rate

Полет в позицию в глобальной системе координат вращаясь с заданной скоростью по рысканью.

Параметры: lat (широта), lon (долгота), z (высота в системе координат frame_id), yaw_rate (угловая скорость по рысканью), update_frame.

### set_velocity

Установить скорости и рысканье. Параметр `frame_id` влияет только на ориентацию результирующего вектора скорости, но не на его длину.

Параметры: vx, vy, vz, yaw, frame_id, update_frame

Полет по кругу:

```python
set_velocity_yaw_rate(vx=0.2, vy=0.0, vz=0, yaw_rate=0.5, frame_id: 'fcu_horiz', update_frame: true)
```

### set_velocity_yaw_rate

Установить скорости и угловую скорость по рысканью.

Параметры: vx, vy, vz, yaw_rate, frame_id, update_frame

### set_attitude

Установить тангаж, крен, рысканье и уровень газа. Имеет смысл использовать этот сервис со значением frame_id равным `fcu_horiz`.

Параметры: pitch, roll, yaw, thrust, frame_id, update_frame

### set_attitude_yaw_rate

Установить тангаж, крен, угловую скорость по рысканью и уровень газа. Имеет смысл использовать этот сервис со значением frame_id равным `fcu_horiz`. **Возможно, не поддерживается в PX4**.

Параметры: pitch, roll, yaw_rate, thrust

### set_rates_yaw

Установить угловые скорости по тангажу и крену, рысканье и уровень газа.

Параметры: pitch_rate, roll_rate, yaw, thrust, frame_id, update_frame

### set_rates

Установить угловые скорости по тагажу, крену и рысканью и уровень газа.

Параметры: pitch_rate, roll_rate, yaw_rate, thrust

### release

Перестать публиковать команды коптеру (отпустить управление).
Возможно продолжение управления средствами MAVROS.

Посадка
-------

Для посадки можно использовать режим ``AUTO.LAND``. Land detector должен быть включен и указан в ``LPE_FUSION``.

```python
from mavros_msgs.srv import SetMode

# ...

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)  # объявляем прокси к сервису переключения режимов

# ...

set_mode(base_mode=0, custom_mode='AUTO.LAND')  # включаем режим посадки
```
