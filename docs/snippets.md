Примеры кода
===

Python
---

> **Note** При использовании кириллических символов в кодировке UTF-8 необходимо добавить в начало программы указание кодировки:
> ```python
> # -*- coding: utf-8 -*-
> ```

---

Функция определения расстяния между двумя точками (**важно**: точки должны быть в одной [системе координат](frames.md)):

```python
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
```

---

Функция для приблизительного определения расстояния (в метрах) между двумя глобальными координатами (широта/долгота):

```python
def get_distance_global(lat1, lon1, lat2, lon2):
    return math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5
```

---

Взлет и ожидание окончания взлета:

```python
z = 2  # высота
tolerance = 0.2  # точность проверки высоты (м)

# Запоминаем изначальную точку
start = get_telemetry()

# Взлетаем на 2 м
print navigate(z=z, speed=0.5, frame_id='fcu_horiz', auto_arm=True)

# Ожидаем взлета
while True:
    # Проверяем текущую высоту
    if get_telemetry().z - start.z + z < tolerance:
        # Взлет завершен
        break
    rospy.sleep(0.2)
```

---

Лететь в точку и ждать пока коптер долетит в нее:

```python
tolerance = 0.2  # точность проверки прилета (м)
frame_id='aruco_map'

# Летим в точку 1:2:3 в поле ArUco-маркеров
print navigate(frame_id=frame_id, x=1, y=2, z=3, speed=0.5)

# Ждем, пока коптер долетит до запрошенной точки
while True:
    telem = get_telemetry(frame_id=frame_id)
    # Вычисляем расстояние до заданной точки
    if get_distance(1, 2, 3, telem.x, telem.y, telem.z) < tolerance:
        # Долетели до необходимой точки
        break
    rospy.sleep(0.2)
```

---

Рассчет общего угла коптера к горизонту:

TODO: fix

```python
telem = get_telemetry()

angle_to_horizon = math.atan(math.hypot(math.tan(telem.pitch), math.tan(telem.roll)))
```

---

Полет по круговой траектории:

```python
RADIUS = 0.6  # m
SPEED = 0.3  # rad / s

start = get_telemetry()
start_stamp = rospy.get_rostime()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    angle = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
    x = start.x + math.sin(angle) * RADIUS
    y = start.y + math.cos(angle) * RADIUS
    set_position(x=x, y=y, z=start.z)

    r.sleep()
```

---

Пример подписки на топики из MAVROS:


```python
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import RCIn

# ...

def state_update(pose):
    # Обработка новых данных о позиции коптера
    pass

# Остальные функции-обработчики
# ...

rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_update)
rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, velocity_update)
rospy.Subscriber('/mavros/battery', BatteryState, battery_update)
rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)
```

Информацию по топикам MAVROS см. по [ссылке](mavros.md).

---

Пример отправки произвольного [MAVLink-сообщения](mavlink.md) коптеру:

```python
# ...

from mavros import mavlink
from pymavlink import mavutil

# ...

mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)

# Отправка сообщения HEARTBEAT:

hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(
             mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
hb_ros_msg = mavlink.convert_to_rosmsg(hb_mav_msg)

mavlink_pub.publish(hb_ros_msg)

```

---

Реакция на переключение режима на пульте радиоуправления (может быть использовано для запуска автономного полета, см. [пример](https://gist.github.com/okalachev/b709f04522d2f9af97e835baedeb806b)):

```python
from mavros_msgs.msg import RCIn

# Вызывается при получении новых данных с пульта
def rc_callback(data):
	# Произвольная реакция на переключение тумблера на пульте
	if data.channels[5] < 1100:
		# ...
		pass
	elif data.channels[5] > 1900:
		# ...
		pass
	else:
		# ...
		pass

# Создаем подписчик на топик с данными с пульта
rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)

rospy.spin()
```

---

Флип:

TODO
