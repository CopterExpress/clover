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

Чтобы запускать связку с Arduino при старте системы автоматически, необходимо убедиться, что в launch-файле Клевера (`~/catkin_ws/src/clever/clever/clever.launch`) включен запуск Arduino rosserial:

```xml
<arg name="arduino" default="true"/>
```

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

Работа с Клевером
---

Набор сервисов и топиков аналогичен обычному набору в `simple_offboard` и `mavros`.

Пример программы, контролирующей коптер по позиции, с использованием сервисов `set_position` и `set_mode`:

```cpp
// Подключение библиотек для работы с rosseral
#include <ros.h>

// Подключение заголовочных файлов сообщений пакета Clever и MAVROS
#include <clever/SetPosition.h>
#include <mavros_msgs/SetMode.h>

using namespace clever;
using namespace mavros_msgs;

ros::NodeHandle nh;

// Объявление сервисов
ros::ServiceClient<SetPosition::Request, SetPosition::Response> setPosition("/set_position");
ros::ServiceClient<SetMode::Request, SetMode::Response> setMode("/mavros/set_mode");

void setup()
{
  // Инициализация rosserial
  nh.initNode();

  // Инициализация сервисов
  nh.serviceClient(setPosition);
  nh.serviceClient(setMode);

  // Ожидание подключение к Raspberry Pi
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("Startup complete");

  // Пользовательская настройка
  // <...>

  // Тестовая программа
  SetPosition::Request sp_req;
  SetPosition::Response sp_res;
  SetMode::Request sm_req;
  SetMode::Response sm_res;

  // Взлет на 2 метра:
  nh.loginfo("Take off");
  sp_req.yaw = 0;
  sp_req.x = 0;
  sp_req.y = 0;
  sp_req.z = 2;
  sp_req.frame_id = "fcu_horiz";
  sp_req.auto_arm = true;
  setPosition.call(sp_req, sp_res);

 // Ждем 5 секунд
  delay(5000);
  
  sp.req.auto_arm = false;

  // Пролет вперед на 3 метра:
  nh.loginfo("Fly forward");
  sp_req.x = 3;
  sp_req.y = 0;
  sp_req.z = 0;
  setPosition.call(sp_req, sp_res);

  // Ждем 5 секунд
  delay(5000);

  // Посадка
  nh.loginfo("Land");
  sm_req.custom_mode = "OFFBOARD";
  setMode.call(sm_req, sm_res);
}

void loop()
{
}
```