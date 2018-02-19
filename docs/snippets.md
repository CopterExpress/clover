Примеры кода
===

Python
---

Функция определения расстяния между двумя точками (**важно**: точки должны быть в одной [системе координат](/docs/frames.md)):

```python
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
```

---

Взлет и ожидание окончания взлета:

```python
tolerance = 0.2  # точность проверки высоты (м)

# Запоминаем изначальную точку
start = get_telemetry()

# Взлетаем на 2 м
print navigate(z=2, speed=0.5, frame_id='fcu_horiz', auto_arm=True)

# Ожидаем взлета
while True:
    # Проверяем текущую высоту
    if get_telemetry().z - start.z < tolerance:
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
    if get_distance(1, 2, 3, telem.x, telem.y, telem.z) < TOLERANCE:
        # Долетели до необходимой точки
        break
    rospy.sleep(0.2)
```

---

Флип:

TODO
