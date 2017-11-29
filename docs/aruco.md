Навигация с использованием ArUco-маркеров
===

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
<arg name="bottom_camera" default="true"/>
```

```xml
<arg name="aruco" default="true"/>
```

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

### Настройка карты ArUco-меток

В качестве карты меток можно использовать автоматически сгенерированный [ArUco-board](https://docs.opencv.org/trunk/db/da9/tutorial_aruco_board_detection.html).

Настройка карты меток производится с помощью файла `~/catkin_ws/src/clever/clever/aruco.launch`. Для использования AruCo-board введите его параметры:

```xml
<node pkg="nodelet" type="nodelet" name="aruco_pose" args="load aruco_pose/aruco_pose nodelet_manager">
    <param name="frame_id" value="aruco_map_raw"/>
    <param name="type" value="gridboard"/> <!-- тип маркерного поля -->
    <param name="markers_x" value="1"/> <!-- количество маркеров по x -->
    <param name="markers_y" value="6"/> <!-- количество маркеров по y -->
    <param name="first_marker" value="240"/> <!-- ID маркера первого маркера (левого верхнего) -->
    <param name="markers_side" value="0.3362"/> <!-- длина стороны маркера в метрах -->
    <param name="markers_sep" value="0.46"/> <!-- растояние между маркерами -->
</node>
```

Если используется карта с нестандартным порядком ID меток, то можно использовать параметр `marker_ids`:

```xml
<rosparam param="marker_ids">[5, 7, 9, 11, 13, 15]</rosparam>
```

Нумерация маркеров ведется с левого верхнего угла поля.

Для контроля карты, по которой в данный момент коптер осуществляет навигацию, можно просмотреть содержимое топика `aruco_pose/map_image`. Через браузер его можно просмотреть при помощи [web_video_server](/docs/web_video_server.md) по ссылке http://192.168.11.1:8080/snapshot?topic=/aruco_pose/map_image:

![](/assets/Снимок экрана 2017-11-27 в 23.20.49.png)

При полетах необходимо убедиться, что наклеенные на пол метки соответствуют карте по ссылке.

В топике `aruco_pose/debug` (http://192.168.11.1:8080/snapshot?topic=/aruco_pose/debug) доступен текущий результат распознования меток:

TODO

### Система координат

По [соглашению](http://www.ros.org/reps/rep-0103.html), в маркерном поле используется стандартная система координат ENU:

* x — вправо (условный "восток");
* y — вверх (условный "север");
* z — вверх.

Таким образом, нулевой является левая нижня точка маркерного поля. Угол по рысканью считается равным 0, когда коптер смотрит направо (по оси x).

![](/assets/aruco-frame.png)

### Настройка полетного контролера

Для правильной работы Vision Position Estimation необходимо (через [QGroundControl](/docs/gcs_bridge.md)) убедиться, что:

* Для PixHawk: Установлена прошивка с LPE (local position estimator). Для PixRacer: параметр `SYS_MC_EST_GROUP` установлен в `local_position_estimator, attitude_estimator_q`.
* В параметре `LPE_FUSION` включены **только** флажки `vision position`, `vision yaw`, `land detector`. Опционально можно включить барометр (baro). 
* Выключен компас: `ATT_W_MAG` = 0
* Включена ориентация по Yaw по зрению: `ATT_EXT_HDG_M` = `Vision`.
* Настройки VPE: `LPE_VIS_DELAY` = 0.03 sec, `LPE_VIS_XY` = 0.1 m, `LPE_VIS_Z` = 0.15 m.
* Рекомендуемые настройки контроллера: 
* * Максимальная скорость по позиции: ``MPC_XY_VEL_MAX`` = 1 m/s
* * Настройки PID-регуляторов: `MPC_XY_P` = 0.12, `MPC_XY_VEL_P` = 0.12.

### Полет

При правильной настройке коптер начнет удерживать позицию по VPE (в [режимах](/docs/modes.md) `POSCTL` или `OFFBOARD`) автоматически.

Для [автономных полетов](/docs/simple_offboard.md) можно будет использовать функции `set_position`, `set_velocity`. Для полета в определенные координаты маркерного поля необхоимо использовать фрейм `aruco_map`:

```python
# Вначале необходимо взлететь, чтобы коптер увидел карту меток
# и появился фрейм aruco_map:
set_position(x=0, y=0, z=3, frame_id='fcu_horiz')  #  взлет на 3 метра

time.sleep(5)

# Полет в координату 2:2 маркерного поля, высота 3 метра
set_position(x=2, y=2, z=3, frame_id='aruco_map', update_frame=True)  #  полет в координату 2:2, высота 3 метра
```