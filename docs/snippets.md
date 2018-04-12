Примеры кода
===

Python
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

Запуск полётной программы с пульта:

```python

#!/usr/bin/python
import rospy
import thread
import sys
import math
from mavros_msgs.msg import RCIn
from clever import srv
from time import sleep
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool

states = ('start','stop','unknown')
state = states[2]

rospy.init_node('Clever3_RC_Script')

navigate = rospy.ServiceProxy('/navigate', srv.Navigate)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
get_telemetry = rospy.ServiceProxy('/get_telemetry', srv.GetTelemetry)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

def get_distance(x1, y1, z1, x2, y2, z2):
	return math.sqrt((x1-x2)**2 + (y1-y2)**2 +(z1-z2)**2)

def takeoff (zp, sp = 1, tolerance = 0.2):
	start = get_telemetry()
	print navigate(z=zp, speed=sp, frame_id='fcu_horiz', auto_arm=True)
	while True:
		telem = get_telemetry()
		delta = abs(abs(telem.z - start.z)-zp)
		if delta < tolerance:
			break
		rospy.sleep(0.2)

def land(sp = 1, land_height = -1, tolerance = 0.25):
	print 'land!'
	z0 = get_telemetry(frame_id='local_origin').z
	print z0	
	h = get_telemetry(frame_id='aruco_map').z
	print h
	print navigate(z=-h+land_height, speed=sp, frame_id='fcu_horiz')
	while True:
		z = get_telemetry(frame_id='local_origin').z
		delta = z0-z-h
		print delta
		if (abs(delta) < tolerance):
			print get_telemetry(frame_id='local_origin')			
			arming(False)
			break
		rospy.sleep(0.2)			

def flight_to_point(xp, yp, zp, sp = 1, breakable = True, tolerance = 0.2, constant_yaw = True):
	frame_id = 'aruco_map'
	if constant_yaw:
		current_yaw = get_telemetry(frame_id = 'aruco_map').yaw
		print navigate(frame_id=frame_id, x=xp, y=yp, z=zp, speed=sp, yaw = current_yaw)
	else:
		print navigate(frame_id=frame_id, x=xp, y=yp, z=zp, speed=sp)
	while True:
		if breakable and state == 'stop':
			return
		telem = get_telemetry(frame_id=frame_id)
		if get_distance(xp, yp, zp, telem.x, telem.y, telem.z) < tolerance:
			break
		rospy.sleep(0.2)

# copter parameters

speed = 1
z = 1

# rectangle parameters

width = 1
height = 1
x0 = 0.1
y0 = 0.6

# flight program

def flight_program (param):
	while True:

		global state
		print 'waiting for stop!'
		while state != 'stop':
		    rospy.sleep(0.1)
		print 'waiting for start...'
		while state == 'stop':
		    rospy.sleep(0.1)
		print 'start!'

		takeoff(z) #takeoff
		flight_to_point(x0, y0, z, speed)
		while True:
			flight_to_point(x0, y0 + height, z, speed)
			flight_to_point(x0 + width, y0 + height, z, speed)
			flight_to_point(x0 + width, y0, z, speed)
			flight_to_point(x0, y0, z, speed, breakable = False)
			if state == 'stop':
				break
		land()

# Вызывается при обновлении данных из топика
def callback(data):
	global state
	# Обрабатываем данные с 6 канала пульта
	if data.channels[5] < 1100:
		state = states[1]
	elif data.channels[5] > 1900:
		state = states[0]
	else:
		state = states[2]	

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

	rospy.Subscriber('mavros/rc/in', RCIn, callback)

    # spin() simply keeps python from exiting until this node is stopped
	
	rospy.spin()

param = []
thread.start_new_thread(flight_program, (param,))
listener()
```
---

Флип:

TODO
