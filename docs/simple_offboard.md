Simple offboard
===

Модуль `simple_offboard` пакета `clever` предназначен для упрощенного программирования автономного дрона (режим `OFFBOARD`). Он позволяет устанавливать желаемые полетные  задачи и автоматически трансформирует [систему координат](/docs/frames.md). 

`simple_offboard` является высокоуровневым способом взаимодействия с полетным контроллером. Для более низкоуровневой работы см. [mavros](/docs/mavros.md).

Основные сервисы – `get_telemetry` (получение всей телеметрии разом), `navigate` (полет в заданную точку по прямой), `land` (переход в режим посадки).

Общие для сервисов параметры:

* `auto_arm` = `true`/`false` – перевести коптер в OFFBOARD и заармить автоматически (**коптер взлетит, если находится на полу!**)
* `frame_id` — система координат в TF2, в которой заданы координаты и рысканье (yaw), [описание систем координат](/docs/frames.md);
* `update_frame` — считать ли систему координат изменяющейся (например, `false` для `local_origin`, `fcu`, `fcu_horiz`, `true` для `marker_map`);
* `x`, `y` – горизонтальные координаты в системе координат `frame_id`;
* `z` — высота в системе координат `frame_id`;
* `lat`, `lon` – широта и долгота (в градусах);
* `yaw` — рысканье в радианах в системе координат `frame_id` (0 – коптер смотрит по оси X);
* `yaw_rate` — угловая скорость по рысканью в радианах в секунду (против часовой), `yaw` должен быть установлен в NaN;
* `thrust` — уровень газа от 0 (нет газа) до 1 (полный газ).

> **Warning** API модуля `simple_offboard` на данный момент нестабилен и может измениться.

Использование из языка Python
---

Пример программы, объявляющей прокси ко всем сервисам:

```python
import rospy
from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('foo')

# Создаем прокси ко всем сервисам:

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_position_global = rospy.ServiceProxy('set_position_global', srv.SetPositionGlobal)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
release = rospy.ServiceProxy('release', Trigger)
```

Неиспользуемые фукнции-прокси можно удалить из кода.

Список сервисов
---

### get_telemetry

Получить полную телеметрию коптера. Параметр: `frame_id` – фрейм для значений `x`, `y`, `z`, `vx`, `vy`, `vz`. Пример: `local_origin`, `fcu_horiz`, `aruco_map`.

Ответ:

* `frame_id` – фрейм
* `connected` – есть ли подключение к <abbr title="Flight Control Unit, полетный контроллер">FCU</abbr>
* `armed` – состояние `armed` винтов (винты включены, если true)
* `mode` - текущий [полетный режим](/docs/modes.md)
* `x, y, z` – позиция коптера в системе координат `frame_id`
* `lat, lon` – текущая широта и долгота (при наличии [gps](/docs/gps.md))
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

Вывод текущей телеметрии из командной строки:

```bash
rosservice call /get_telemetry "{frame_id: ''}"
```

### navigate

Прилететь в обозначенную точку по прямой.

Параметры:

* `x`, `y`, `z` – координаты в системе `frame_id`
* `yaw` – угол по рысканью
* `yaw_rate` – угловая скорость по рысканью (при установке yaw в NaN)
* `speed` – скорость полета (скорость движения setpoint)
* `frame_id`, `update_frame`, `auto_arm`.

Примеры:

```python
# плавно взлететь на высоту 1.5 м со скоростью взлета 0.5 м/с
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='fcu_horiz', auto_arm=True)
```

```python
# прилететь по прямой в точку 5:0 (высота 2)
# в локальной системе координат со скоростью 0.8 м/с
navigate(x=5, y=0, z=3, speed=0.8)
```


```python
# пролететь вправо относительно коптера на 3 м
navigate(x=0, y=-1, z=0, speed=1, frame_id='fcu_horiz')
```

```python
# прилететь в точку 3:2 (высота 2) в системе координат маркерного поля
# со скоростью 1 м/с
navigate(x=3, y=2, z=2, speed=1, frame_id='aruco_map', update_frame=True)
```

Пример взлета на коптере на 2 метра из командной строки:

```bash
rosservice call /navigate "{x: 0.0, y: 0.0, z: 2, yaw: 0.0, speed: 0.5, frame_id: 'fcu_horiz', update_frame: false, auto_arm: true}"
```

### navigate_global

> **Info** Образ версии >0.5.

Полет по прямой в точку в глобальной системе координат (широта/долгота).

Параметры:

* `lat`, `lon` – широта и долгота;
* `z` – высота в системе координат `frame_id`;
* `yaw` – угол по рысканью;
* `yaw_rate` – угловая скорость по рысканью (при установке yaw в NaN);
* `speed` – скорость полета (скорость движения setpoint);
* `frame_id`, `update_frame`, `auto_arm`.

Объявление прокси к сервису:

```python
navigate_global = rospy.ServiceProxy('/navigate_global', srv.NavigateGlobal)
```

Полет в глобальную точку по прямой (оставаясь на текущей высоте):

```python
navigate_global(lat=55.707033, lon=37.725010, z=0, frame_id='fcu_horiz')
```

Пример полета в глобальную точку из командной строки:

```bash
rosservice call /navigate_global "{lat: 55.707033, lon: 37.725010, z: 0.0, yaw: 0.0, speed: 3.0, frame_id: 'fcu_horiz', update_frame: false, auto_arm: false}"
```

### set_position

Установить цель по позиции и рысканью.

Параметры:

* `x`, `y`, `z` – координаты точки в системе координат `frame_id`;
* `yaw` – угол по рысканью;
* `yaw_rate` – угловая скорость по рысканью (при установке yaw в NaN);
* `speed` – скорость полета (скорость движения setpoint);
* `frame_id`, `update_frame`, `auto_arm`.

> **Hint** Для полета на точку по прямой или взлета используйте более высокоуровневый сервис `navigate`.

Задание позиции относительно текущей позиции коптера:

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

Вращение на месте со скоростью 0.5 рад/c:

```python
set_position(x=0, y=0, z=0, frame_id='fcu_horiz', yaw=float('nan'), yaw_rate=0.5)
```

### set_velocity

Установить скорости и рысканье.

* `vx`, `vy`, `vz` – требуемая скорость полета;
* `yaw` – угол по рысканью;
* `yaw_rate` – угловая скорость по рысканью (при установке yaw в NaN);
* `frame_id`, `update_frame`, `auto_arm`.

> **Note** Параметр `frame_id` влияет только на ориентацию результирующего вектора скорости, но не на его длину.

Параметры: vx, vy, vz, yaw, frame_id, update_frame

Полет вперед (относительно коптера) со скоростью 1 м/с:

```python
set_velocity(vx=1, vy=0.0, vz=0, frame_id: 'fcu_horiz')
```

Полет по кругу:

```python
set_velocity(vx=0.2, vy=0.0, vz=0, yaw=float('nan'), yaw_rate=0.5, frame_id: 'fcu_horiz', update_frame: True)
```

### set_attitude

Установить тангаж, крен, рысканье и уровень газа.

> **Note** Параметр `frame_id` определяет систему координат, в которой задается `yaw`.

Параметры:

* `pitch`, `roll`, `yaw` – необходимый угол по тангажу, крену и рысканью (рад.);
* `thrust` – уровень газа от 0 (нет газа) до 1 (полный газ);
* `frame_id`, `update_frame`.

### set_rates

Установить угловые скорости по тагажу, крену и рысканью и уровень газа.

Параметры:

* pitch_rate, roll_rate, yaw_rate – угловая скорость по танажу, крену и рыканью (рад/с);
* thrust – уровень газа от 0 (нет газа) до 1 (полный газ).

### release

Перестать публиковать sepoint'ы коптеру (отпустить управление).
Возможно продолжение управления средствами [MAVROS](/docs/mavros.md), [Веб-пультом управления](/docs/web_rc.md).

Посадка
-------

> **Info** Образ версии >0.5.

Для посадки можно использовать сервис `/land`. При вызове сервиса коптер автоматически переведется в [режим](/docs/modes.md) `AUTO.LAND` (или аналогичный).

Объявление прокси к сервису:

```python
land = rospy.ServiceProxy('/land', Trigger)
```

Посадка коптера:

```python
res = land()

if res.success:
    # коптер успешно переведен в режим AUTO.LAND
    # ...
```

Пример использования сервиса из командной строки:

```bash
rosservice call /land "{}"
```

> **Note** Для автоматического отключения винтов после посадки PX4-параметр `COM_DISARM_LAND` должен быть установлен в значение > 0.

В предущих версиях для посадки необходимо перевести коптер в режим ``AUTO.LAND``, используя mavros.

```python
from mavros_msgs.srv import SetMode

# ...

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)  # объявляем прокси к сервису переключения режимов

# ...

set_mode(base_mode=0, custom_mode='AUTO.LAND')  # включаем режим посадки
```

Для полетов в поле ArUco-макеров см. [навигация по ArUco](/docs/aruco.md).
