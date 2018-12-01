Использование QGroundControl через Wi-Fi
===

![QGroundControl](../assets/qground.png)

Возможны контроль, управление, калибровка и настройка полетного контроллера квадрокоптера с помощью программы QGroundControl по Wi-Fi.
Для этого необходимо [подключиться к Wi-Fi](wifi.md) сети `CLEVER-xxxx`.

После чего в launch-файле Клевера `/home/pi/catkin_ws/src/clever/clever/launch/clever.launch` выбрать один из преднастроенных режимов бриджа.

После изменения launch-файла необходимо перезагрузить сервис clever:

```(bash)
sudo systemctl restart clever
```

TCP-бридж
---

Изменить параметр `gcs_bridge` в launch-файле:

```xml
<arg name="gcs_bridge" default="tcp"/>
```

Затем в программе QGroundControl нужно выбрать Application Settings > Comm Links > Add. Создать подключение со следующими настройками:

![QGroundControl TCP connection](../assets/bridge_tcp.png)

Затем необходимо выбрать в списке подключений "Clever" и нажать "Connect".

UDP бридж (с автоматическим подключением)
---

Изменить параметр gcs_bridge в launch-файле:

```xml
<arg name="gcs_bridge" default="udp-b"/>
```

При открытии программы QGroundControl соединение должно установиться автоматически.


UDP-бридж (без автоматического подключения)
---

Изменить параметр `gcs_bridge` в launch-файле:

```xml
<arg name="gcs_bridge" default="udp"/>
```

Затем в программе QGroundControl нужно выбрать Application Settings > Comm Links > Add. Создать подключение со следующими настройками:

![QGroundControl UDP connection](../assets/bridge_udp.png)

Затем необходимо выбрать в списке подключений "CLEVER" и нажать "Connect".

UDP broadcast-бридж
---

> **Hint** Особенностью UDP broadcast-бриджа является возможность просмотра телеметрии дрона одновременно с нескольких устройств (например с телефона и компьютера). Также он хорошо подходит для организации сети из устройств при помощи роутера.

Изменить параметр `gcs_bridge` в launch-файле:

```xml
<arg name="gcs_bridge" default="udp-pb"/>
```

При открытии программы QGroundControl соединение должно установиться автоматически.
