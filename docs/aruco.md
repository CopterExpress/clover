Навигация с использованием ArUco-маркеров
===

TODO

[ArUco-маркеры](https://docs.opencv.org/3.2.0/d5/dae/tutorial_aruco_detection.html) — это популярная технология для позиционирования 
роботехнических систем с использованием компьютерного зрения.

Пример ArUco-маркеров:

![](/assets/markers.jpg)

aruco_pose
---

Модуль `aruco_pose` позволяет восстанавливать позицию коптера относительно карты ArUco-маркеров и сообщать ее полетному контролеру, используя механизм [Vision Position Estimation](https://dev.px4.io/en/ros/external_position_estimation.html).

При наличия источника положения коптера по маркерам, появляется возможность производить точную автономную indoor-навигацию по позициям при помощи модуля [simple_offboard](/docs/simple_offboard.md).

### Включение

Необходимо убедиться, что в launch-файле Клевера (`~/catkin_ws/src/clever/clever/clever.launch`) включен запуск aruco_pose и нижней камеры для компьютерного зрения:

```xml
<arg name="camera" default="true"/>
```

```xml
<arg name="aruco_pose" default="true"/>
```

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

### Настройка карты ArUco-меток

В качестве карты меток можно использовать автоматически сгенерированный [ArUco-board](https://docs.opencv.org/trunk/db/da9/tutorial_aruco_board_detection.html).

TODO

Для контроля карты, по которой в данный момент коптер осуществляет навигацию, можно просмотре содержимое топика `aruco_pose/map_image`. Через браузер его можно просмотреть при помощи [web_video_server](/docs/web_video_server.md) по ссылке http://192.168.11.1:8080/snapshot?topic=/aruco_pose/map_image:

![](/assets/Снимок экрана 2017-11-27 в 23.20.49.png)

При полетах необходимо убедиться, что наклеенные на пол метки соответствуют карте по ссылке.

В топике `aruco_pose/debug` (http://192.168.11.1:8080/snapshot?topic=/aruco_pose/debug) доступен текущий результат распознования меток:

TODO

### Настройка полетного контролера

Для правильной работы Vision Position Estimation необходимо (через [QGroundControl](/docs/gcs_bridge.md)) убедиться, что:

* Установлена прошивка с LPE (local position estimator).
* В параметре `LPE_FUSION` включены **только** флажки `vision position`, `vision yaw`, `land detector`. Опционально можно включить барометр (baro). 
* Выключен компас: `ATT_W_MAG` = 0
* Включена ориентация по Yaw по зрению: `ATT_EXT_HDG_M` = `Vision`.

