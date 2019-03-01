# Навигация по картам ArUco-маркеров

> **Info** Для распознавания маркеров модуль камеры должен быть корректно подключен и [сконфигурирован](camera.md).

Модуль `aruco_map` распознает карты ArUco-маркеров, как единое целое. Также возможна навигация по картам ArUco-маркеров с использованием механизма Vision Position Estimate (VPE).

## Конфигурирование

Для включения распознавания карт маркеров аргумент `aruco_map` в файле `~/catkin_ws/src/clever/clever/launch/aruco.launch` должен быть в значении `true`:

```xml
<arg name="aruco_map" default="true"/>
```

Для включения передачи координат в полетный контроллер по механизму VPE, аргумента `aruco_vpe` должен быть в значении `true`:

```xml
<arg name="aruco_vpe" default="true"/>
```

## Настройка карты маркеров

Карта загружается из текстового файла, каждая строка которого имеет следующий формат:

```
id_маркера размер_маркера x y z угол_z угол_y угол_x
```

Где `угол_N` – это угол поворота маркера вокруг оси N в радианах.

Путь к файлу с картой задается в параметре `map`:

```xml
<param name="map" value="$(find aruco_pose)/map/map.txt"/>
```

Смотрите примеры карт маркеров в каталоге [`~/catkin_ws/src/clever/aruco_pose/map`](https://github.com/CopterExpress/clever/tree/master/aruco_pose/map).

### Проверка

Для контроля карты, по которой в данный момент коптер осуществляет навигацию, можно просмотреть содержимое топика `/aruco_map/image`. Через браузер его можно просмотреть при помощи [web_video_server](web_video_server.md) по ссылке http://192.168.11.1:8080/snapshot?topic=/aruco_map/image:

<img src="../assets/aruco-map.png" width=600>

Клевер публикует текущую позицию распознанной карты в топик `aruco_map/pose`. Также публикуется [TF-фрейм](frames.md) `aruco_map` (VPE выключен) или `aruco_map_detected` (VPE включен). Используя топик `aruco_map/visualization` можно визуализировать текущую карту маркеров в [rviz](rviz.md).

Наглядно позиция распознанной карты отображается в топике `aruco_map/debug` (просмотр доступен по ссылке http://192.168.11.1:8080/stream_viewer?topic=/aruco_map/debug):

<img src="../assets/aruco-map-debug.png" width=600>

## Система координат

По [соглашению](http://www.ros.org/reps/rep-0103.html) в маркерном поле используется стандартная система координат <abbr title="East-North-Up">ENU</abbr>:

* ось **<font color=red>x</font>** указывает кверху карты маркеров;
* ось **<font color=green>y</font>** указывает на правую сторону карты маркеров;
* ось **<font color=blue>z</font>** указывает от плоскости карты маркеров.

<img src="../assets/aruco-map-axis.png" width="600">

## Настройка VPE

Для работы механизма Vision Position Estimation необходимы следующие [настройки PX4](px4_parameters.md).

При использовании **EKF2** (параметр `SYS_MC_EST_GROUP` = `ekf2`):

* В параметре `EKF2_AID_MASK` включены флажки `vision position fusion`, `vision yaw fusion`.
* Шум угла по зрению: `EKF2_EVA_NOISE` = 0.1 rad
* Шум позиции по зрению: `EKF2_EVP_NOISE` = 0.05 m
* `EKF2_EV_DELAY` = 0

При использовании **LPE** (параметр `SYS_MC_EST_GROUP` = `local_position_estimator, attitude_estimator_q`):

* В параметре `LPE_FUSION` включены флажки `vision position`, `land detector`.
* Вес угла по рысканью по зрению: `ATT_W_EXT_HDG` = 0.5
* Включена ориентация по Yaw по зрению: `ATT_EXT_HDG_M` = 1 `Vision`.
* Шумы позиции по зрению: `LPE_VIS_XY` = 0.05 m, `LPE_VIS_Z` = 0.05 m.
* `LPE_VIS_DELAY` = 0 sec

<!-- * Выключен компас: `ATT_W_MAG` = 0 -->

> **Info** Для использования LPE в Pixhawk необходимо [скачать прошивку с названием `px4fmu-v2_lpe.px4`](https://github.com/PX4/Firmware/releases).

## Полет

При правильной настройке коптер начнет удерживать позицию в [режимах](modes.md) `POSCTL` и `OFFBOARD` автоматически.

Для [автономных полетов](simple_offboard.md) можно будет использовать функции `navigate`, `set_position`, `set_velocity`. Для полета в определенные координаты маркерного поля необходимо использовать фрейм `aruco_map`:

```python
# Вначале необходимо взлететь, чтобы коптер увидел карту меток и появился фрейм aruco_map:
navigate(0, 0, 2, frame_id='body', speed=0.5, auto_arm=True)  # взлет на 2 метра

time.sleep(5)

# Полет в координату 2:2 маркерного поля, высота 2 метра
navigate(2, 2, 2, speed=1, frame_id='aruco_map')  # полет в координату 2:2, высота 3 метра
```

## Дополнительные настройки

Если коптер нестабильно удерживает позицию по VPE, попробуйте увеличить коэффициенты *P* PID-регулятора по скорости – параметры `MPC_XY_VEL_P` и `MPC_Z_VEL_P`.

## Расположение маркеров на потолке

![Маркеры на потолке](../assets/IMG_4175.JPG)

Для навигации по маркерам, расположенным на потолке, необходимо поставить основную камеру так, чтобы она смотрела вверх и [установить соответствующий фрейм камеры](camera_frame.md).

Также в файле `~/catkin_ws/src/clever/clever/launch/aruco.launch` необходимо установить параметр `known_tilt` в секции `aruco_map` в значение `map_flipped`:

```xml
<param name="known_tilt" value="map_flipped"/>
```

При такой конфигурации фрейм `aruco_map` также окажется перевернутым. Таким образом, для полета на высоту 2 метра ниже потолка, аргумент `z` нужно устанавливать в 2:

```python
navigate(x=1, y=2, z=1.1, speed=0.5, frame_id='aruco_map')
```
