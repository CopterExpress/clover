Управление коптером с Arudino
===

Для взаимодействия с ROS-топиками и сервисами на Raspberry Pi можно использовать библиотеку [rosserial_arduino](http://wiki.ros.org/rosserial_arduino).

Основной туториал: http://wiki.ros.org/rosserial_arduino/Tutorials

Arudino необходимо установить на Клевер и подключить по USB-порту.

Настройка Aruino IDE
---

Необходимо скачать и скопировать [библиотеку ROS-сообщений Клевера](https://github.com/CopterExpress/clever_bundle/blob/master/deploy/clever_arudino.tar.gz?raw=true) (`ros_lib`) в `<папку скетчей>/libraries`.

Настройка Raspberry Pi
---

Чтобы единоразово запустить программу на Arduino, можно воспользоваться командой:

```
roslaunch clever arduino.launch
```

Чтобы запускать связку с Arduino при старте системы автоматически, необходимо установить аргумент `arudino` в launch-файле Клевера (`~/catkin_ws/src/clever/clever/clever.launch`):

```xml
<arg name="arduino" default="true"/>
```

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

Работа с Клевером
---

Набор сервисов и топиков аналогичен обычному набору в [simple_offboard](/docs/simple_offboard.md) и [mavros](/docs/mavros.md).

Пример программы, контролирующей коптер по позиции, с использованием сервисов `set_position` и `set_mode`:

```cpp
// Подключение библиотек для работы с rosseral
#include <ros.h>

// Подключение заголовочных файлов сообщений пакета Clever и MAVROS
#include <clever/Navigate.h>
#include <clever/SetPosition.h>
#include <mavros_msgs/SetMode.h>

using namespace clever;
using namespace mavros_msgs;

ros::NodeHandle nh;

// Объявление сервисов
ros::ServiceClient<Navigate::Request, Navigate::Response> navigate("/navigate");
ros::ServiceClient<SetPosition::Request, SetPosition::Response> setPosition("/set_position");
ros::ServiceClient<SetMode::Request, SetMode::Response> setMode("/mavros/set_mode");

void setup()
{
  // Инициализация rosserial
  nh.initNode();

  // Инициализация сервисов
  nh.serviceClient(navigate);
  nh.serviceClient(setPosition);
  nh.serviceClient(setMode);

  // Ожидание подключение к Raspberry Pi
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("Startup complete");

  // Пользовательская настройка
  // <...>

  // Тестовая программа
  Navigate::Request nav_req;
  Navigate::Response nav_res;
  SetPosition::Request sp_req;
  SetPosition::Response sp_res;
  SetMode::Request sm_req;
  SetMode::Response sm_res;

  // Взлет на 2 метра:
  nh.loginfo("Take off");
  nav_req.auto_arm = false;
  nav_req.x = 0;
  nav_req.y = 0;
  nav_req.z = 2;
  nav_req.frame_id = "fcu_horiz";
  nav_req.speed = 0.5;
  navigate.call(nav_req, nav_res);

 // Ждем 5 секунд
  delay(5000);
  
  sp.req.auto_arm = false;

  // Пролет вперед на 3 метра:
  nh.loginfo("Fly forward");
  nav_req.auto_arm = false;
  nav_req.x = 3;
  nav_req.y = 0;
  nav_req.z = 0;
  nav_req.frame_id = "fcu_horiz";
  nav_req.speed = 0.8;
  navigate.call(nav_req, nav_res);
  
  // Полет в точку 1:0:2 по маркерному полю
  nh.loginfo("Fly on point");
  nav_req.auto_arm = false;
  nav_req.x = 1;
  nav_req.y = 0;
  nav_req.z = 2;
  nav_req.frame_id = "aruco_map";
  nav_req.update_frame = true;
  nav_req.speed = 0.8;
  navigate.call(nav_req, nav_res);

  // Ждем 5 секунд
  delay(5000);

  // Посадка
  nh.loginfo("Land");
  sm_req.custom_mode = "AUTO.LAND";
  setMode.call(sm_req, sm_res);
}

void loop()
{
}
```