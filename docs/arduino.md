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

Необходимо убедиться с в launch-файле Клевера (`~/catkin_ws/src/clever/clever/clever.launch`) включен запуск Arduino rosserial:

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

Пример кода контроля коптера по позиции:

```c++
```

Пример кода с объявлением прокси ко всем сервисам пакета Clever:
