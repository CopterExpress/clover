Навигация с использованием ArUco-маркеров
===

[ArUco-маркеры](https://docs.opencv.org/3.2.0/d5/dae/tutorial_aruco_detection.html) — это популярная технология для позиционирования 
роботехнических систем с использованием компьютерного зрения.

Пример ArUco-маркеров:

![](/assets/markers.jpg)

aruco_pose
---

Модуль `aruco_pose` позволяет восстанавливать позицию коптера относительно карты ArUco-маркеров и сообщать ее полетному контролеру, используя механизм [Vision Position Estimation](https://dev.px4.io/en/ros/external_position_estimation.html).

При наличии источника положения коптера по маркерам появляется возможность производить точную автономную indoor-навигацию по позициям при помощи модуля [simple_offboard](/docs/simple_offboard.md).

### Включение

Необходимо убедиться, что в launch-файле Клевера (`~/catkin_ws/src/clever/clever/launch/clever.launch`) включен запуск aruco_pose и [камеры для компьютерного зрения](/docs/camera.md):

```xml
<arg name="main_camera" default="true"/>
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
    <!-- тип маркерного поля -->
    <param name="type" value="gridboard"/>
    
    <!-- количество маркеров по x -->
    <param name="markers_x" value="1"/>
    
    <!-- количество маркеров по y -->
    <param name="markers_y" value="6"/>
    
    <!-- ID маркера первого маркера (левого верхнего) -->
    <param name="first_marker" value="240"/>

    <!-- длина стороны маркера в метрах -->
    <param name="markers_side" value="0.3362"/>
    
    <!-- растояние между маркерами -->
    <param name="markers_sep" value="0.46"/>
</node>
```

Можно задать отдельно расстояние между маркерами по горизонтали и вертикали:
```xml
<!-- расстояние между маркерами по горизонтали -->
<param name="markers_sep_x" value="0.97"/>

<!-- расстояние между маркерами по вертикали -->
<param name="markers_sep_y" value="1.435"/>
```

Если используется карта с нестандартным порядком ID меток, то можно использовать параметр `marker_ids`:

```xml
<rosparam param="marker_ids">[5, 7, 9, 11, 13, 15]</rosparam>
```

Нумерация маркеров ведется с левого верхнего угла поля.

Для контроля карты, по которой в данный момент коптер осуществляет навигацию, можно просмотреть содержимое топика `aruco_pose/map_image`. Через браузер его можно просмотреть при помощи [web_video_server](/docs/web_video_server.md) по ссылке http://192.168.11.1:8080/snapshot?topic=/aruco_pose/map_image:

![](/assets/Снимок экрана 2017-11-27 в 23.20.49.png)

При полетах необходимо убедиться, что наклеенные на пол метки соответствуют карте.

В топике `aruco_pose/debug` (http://192.168.11.1:8080/snapshot?topic=/aruco_pose/debug) доступен текущий результат распознования меток:

TODO

### Система координат

По [соглашению](http://www.ros.org/reps/rep-0103.html), в маркерном поле используется стандартная система координат ENU:

* x — вправо (условный "восток");
* y — вперед (условный "север");
* z — вверх.

_Примечание_: указанное выше определение приведено для ситуации, когда поле маркеров лежит на полу.

Таким образом, нулевой является левая нижня точка маркерного поля. Угол по рысканью считается равным 0, когда коптер смотрит направо (по оси x).

![](/assets/aruco-frame.png)

### Настройка полетного контролера

Для правильной работы Vision Position Estimation необходимо (через [QGroundControl](/docs/gcs_bridge.md)) убедиться, что:

* Для PixHawk: Установлена прошивка с LPE (local position estimator). Для PixRacer: параметр `SYS_MC_EST_GROUP` установлен в `local_position_estimator, attitude_estimator_q`.
* В параметре `LPE_FUSION` включены **только** флажки `vision position`, `vision yaw`, `land detector`. Итоговое значение _28_.
* Выключен компас: `ATT_W_MAG` = 0
* Вес угла по рысканью по зрению: `ATT_W_EXT_HDG` = 0.5
* Включена ориентация по Yaw по зрению: `ATT_EXT_HDG_M` = `Vision`.
* Настройки VPE: `LPE_VIS_DELAY` = 0 sec, `LPE_VIS_XY` = 0.1 m, `LPE_VIS_Z` = 0.15 m.
* Рекомендуемые настройки контроллера: 
  * Максимальная скорость по позиции: ``MPC_XY_VEL_MAX`` = 3 m/s
  * Настройки PID-регуляторов: `MPC_XY_P` = 0.95, `MPC_XY_VEL_P` = 0.15.
* Рекомендуемые настройки land detector'а:
  * `LNDMC_ROT_MAX` = 45 deg
  * `LNDMC_THR_RANGE` = 0.5
  * `LNDMC_Z_VEL_MAX` = 1 m/s.
  

### Полет

При правильной настройке коптер начнет удерживать позицию по VPE (в [режимах](/docs/modes.md) `POSCTL` или `OFFBOARD`) автоматически.

Для [автономных полетов](/docs/simple_offboard.md) можно будет использовать функции `navigate`, `set_position`, `set_velocity`. Для полета в определенные координаты маркерного поля необходимо использовать фрейм `aruco_map`:

```python
# Вначале необходимо взлететь, чтобы коптер увидел карту меток
# и появился фрейм aruco_map:
navigate(0, 0, 2, frame_id='fcu_horiz', speed=0.5, auto_arm=True) #  взлет на 2 метра

time.sleep(5)

# Полет в координату 2:2 маркерного поля, высота 2 метра
navigate(2, 2, 2, speed=1, frame_id='aruco_map', update_frame=True)  #  полет в координату 2:2, высота 3 метра
```
См. [другие функции](/docs/simple_offboard.md) simple offboard.

### Расположение маркеров на потолке

> **Info** Образ версии >0.2.

Для навигации по маркерам, расположенным на потолке, необходимо поставить основную камеру так, чтобы она смотрела вверх и [установить соответствующий фрейм камеры](/docs/camera_frame.md).

Чтобы задавать карту маркеров в "перевернутой" системе координат, необходимо изменить параметр `aruco_orientation` в файле `~/catkin_ws/src/clever/clever/aruco.launch`:

```xml
<param name="aruco_orientation" value="local_origin_upside_down"/>
```

При задании вышеуказанного параметр, фрейм aruco_map также окажется "перевернутым". Таким образом, для полета на высоту 2 метра ниже потолка, `z` нужно указать в 2:

```python
navigate(x=1, y=2, z=1.1, speed=0.5, frame_id='aruco_map')
```
