Simple offboard
===

Модуль `simple_offboard` пакета `clever` предназначен для упрощенного программирования автономного дрона (режим `OFFBOARD`). Модуль автомтически трансформирует систему координат. При вызове любого из сервисов коптер автоматически переведется в OFFBOARD и заармится (**коптер взлетит, если находится на полу!**)

[Подробнее о системах координат](/docs/frames.md).

Общие для сервисов параметры:

* `frame_id` — система координат в TF2, в которой заданы координаты и рысканье (yaw).
* `update_frame` — считать ли систему координат изменяющейся (например, `false` для `local_origin`, `fcu`, `fcu_horiz`, `true` для `marker_map`)
* `yaw` — рысканье в радианах в заданой системе координат (0 – коптер смотрит по оси X).
* `yaw_rate` — угловая скорость по рысканью в радианах в секунду (против часовой).
* `thrust` — уровень газа (от 0 [нет газа] до 1 [полный газ])

Использование из языка Python
---

Объявление прокси ко всем сервисам:

```python
import rospy
from marker_navigator.srv import SetPosition, \
    SetPositionYawRate, \
    SetPositionGlobal, \
    SetPositionGlobalYawRate, \
    SetVelocity, \
    SetVelocityYawRate, \
    SetAttitude, \
    SetAttitudeYawRate, \
    SetRatesYaw, \
    SetRates
from std_srvs.srv import Trigger

rospy.init_node('foo')

# Создаем прокси ко всем сервисам:

set_position = rospy.ServiceProxy('/set_position', SetPosition)
set_position_yaw_rate = 
rospy.ServiceProxy('/set_position/yaw_rate', SetPositionYawRate)

set_position_global = rospy.ServiceProxy('/set_position_global', SetPosition)
set_position_global_yaw_rate = rospy.ServiceProxy('/set_position_global/yaw_rate', SetPositionYawRate)


set_velocity = rospy.ServiceProxy('/set_velocity', SetVelocity)
set_velocity_yaw_rate = rospy.ServiceProxy('/set_Velocity/yaw_rate', SetVelocityYawRate)

set_attitude = rospy.ServiceProxy('/set_attitude', SetAttitude)
set_attitude_yaw_rate = rospy.ServiceProxy('/set_attitude/yaw_rate', SetattitudeYawRate)

set_rates_yaw = rospy.ServiceProxy('/set_rates/yaw', SetRatesYaw)
set_rates = rospy.ServiceProxy('/set_rates', SetRates)

release = rospy.ServiceProxy('/release', Trigger)
```
