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
    t = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
    x = start.x + math.sin(t) * RADIUS
    y = start.y + math.cos(t) * RADIUS
    set_position(x=x, y=y, z=start.z)

    r.sleep()
```

---

Реакция на переключение режима на пульте радиоуправления (к примеру, может быть использовано для запуска автономного полета, см. [пример](https://gist.github.com/okalachev/b709f04522d2f9af97e835baedeb806b)):

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
