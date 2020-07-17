# Innopolis Open 2020


## Команда 
<ul style="list-style: none; font-size:15px; padding-left:10px">
 <li><b>Юрьев Василий</b></li>
 <li><b>Оконешников Дмитрий</b></li>
</ul>

## Описание задачи финала

Внедрение новых технологий происходит в различных отраслях экономики, в том числе и в сельском хозяйстве. Дроны или БПЛА не стали исключением. Благодаря применению беспилотников оценка состояния сельскохозяйственных территорий и анализ компонентов ландшафта стали более доступными и эффективными. 


<div style="display: inline-block; vertical-align: top;">
<p align="center"><img src="https://lh3.googleusercontent.com/EC9SW_yuIq46aqxf6jpyVJhft5Fjxbned75RMcv_XLYABa6Qobz1fZRM1J0SzlMGRw_cpCa2Z6wyH-uG5r9TNzauScajpE0VxXtNL72UN1aSMs5hL1a3cpjw4L6mIkMia7zpKZIem5vRsc6YxLLcOl7hNknlfcMzPk8GZ7k11IfZzodiPCEjasy_sz4NGsXN0t7r207DQkJdkTCCkOonBNItijZQpPMIFfdDoS2wj8pGrl4r5cmWzWz2T0SEnfkgq_NypHYluFUMMCO1AVgJEbPu9bh5dhd-Vmqy56q12EV8ZAy7ASd-loL_THEos1kWhYMe4mq3UCzTj46E20uBd2MYUIBX1Vff-mug__CGVEJQlMsSolpHG6lw-Z8LufgMBOgIma-a8SmqQyZykyFDUFZ4crDeJwKBYJ0jAdKPSOGhE7LBznajePEFdu_vZKyZuwm0qSim9e5vvNuSUqZcaYL1dOxScabbu0mPgS05qVhnEkcSH_4eZ38hm95D3xj0Ry0qbemf7fmsFk40R8ud8_OZ88lnPFa8C64L7YXvtBt-MZ87DsaLrEEwZMblqSLqr5oXQtNAs-aH5KOtYCSLp8Dk0ZLLaJ9cbsS-WALlGnKg24sUgfWkf3Y_IflsyaYHahbOdZwd7whdEqEvjC2uYAdibDZdmUO0ed_0iTyN99x7VoRdtzspb4s-s6zTE4jx5EHzwQ=w1920-h625-ft" width="300"></p>  
<p><i>Один из вариантов поля</i></p>
</div>
<div style="display: inline-block; vertical-align: top;padding-left:50px; width:40%">
<p>
Финальная задача Innopolis Open 2020 была посвящена мониторингу сельскохозяйственных территорий и состояла из следующих элементов:
</p>
<ul style="list-style-type: decimal; font-size:15px; padding-left:10px">
 <li>Взлет (с QR-кода) и посадка (на цветной маркер небольшого размера).</li>
 <li>Распознавание зашифрованного сообщения в QR-коде.</li>
 <li>Распознавание цвета объектов (цветных маркеров – условное обозначение сельхоз угодий).</li>
 <li>Определение их координат (расположение на поле изменяется).</li>
 <li>Отчет по полученным данным.</li>
</ul>
</div>

# Код
## Github [https://github.com/vas0x59/ior2020_uav_L22_AERO](https://github.com/vas0x59/ior2020_uav_L22_AERO)

## Основной код
При реализации кода в первоначальной концепции использовались свои типы сообщений, множество нод и других возможностей ROS, для обеспечения этого функционала необходимо создавать пакет и компилировать его, но из-за специфики соревнований (использование одной sd карты для все команд) весь код был объединен в один файл. Данный подход усложнил отладку, но упростил запуск на площадке.

Элементы программы:

1. Взлет 
2. Распознавание QR-Кода
3. Поиск цветных маркеров
4. Посадка
5. Генерация отчета и видео 

<!-- Итоговые координаты маркеров основываються на полученных из системы распознавания за весь полет, которые автоматически группируються и усредняються. -->
Итоговыми координатами маркеров являются автоматически сгруппированные и усредненные данные из системы распознавания полученных за весь полет.
Для покрытия всей территории была выбрана траектория "Зиг-заг".
Для отладки применен симулятор Gazebo.

## Цветные маркеры
```l22_aero_vision/src/color_r_c.py```

Для обработки изображения с камеры и детектирования объектов мы использовали функции из библиотеки OpenCV.

Алгоритм

1. Получение изображения и параметров камеры
1. Построение маски по определенному диапазону цветов (в формате HSV)
1. Детектирование контуров цветных объектов
1. Определение типа объекта, получение ключевых точек объекта на изображении
1. Определение положения квадратов и кругов с помощью solvePnP основываясь на реальных размерах объектов и точках на изображении ( [OpenCV Docs](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d) )
1. Отправка результата в топики ```/l22_aero_color/markers```  и  ```/l22_aero_color/circles``` ( координаты относительно main_camera_optical )

Во время разработки были созданы свои типы сообщений, а также сервис для настройки параметров детектора во время посадки. (ColorMarker, ColorMarkerArray, SetParameters). 
Для определения положения цветных объектов в системе координат поля была использована библиотека TF ([http://wiki.ros.org/tf](http://wiki.ros.org/tf))

Из-за искажений по краям изображения от fisheye объектива все распознанные контуры находящийся рядом с краем изображения игнорируются.
Во время посадки данный фильтр отключается.
Определение типа объекта производиться с помощью функций анализа контуров (approxPolyDP - кол-во вершин; minAreaRect, counturArea - соотношение площади описанного квадрата и площади контура + соотношение сторон).

<!-- IMAGE or VIDEO -->
<img src="https://github.com/vas0x59/ior2020_uav_L22_AERO_info/raw/master/to_Gitbook/content/5_D1_2.png" height="355">
<!-- <video id="v1" autoplay preload="auto" loop height="360" controls src="https://github.com/vas0x59/ior2020_uav_L22_AERO_info/raw/master/to_Gitbook/content/IO_UAV_Day2_with_Landing_150x.mp4">
<source id="vs1" ></source>
</video> -->
<iframe width="600" height="360" src="https://www.youtube.com/embed/kCW87RTA838" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<p><i>Примеры распознавание маркеров</i></p>


## Визуализация в RViz
```l22_aero_vision/src/viz.py```

Для отладки распознавания объектов создан скрипт визуализирующий координаты маркеров в среде RViz.

<!-- IMAGE or VIDEO -->

<iframe width="560" height="315" src="https://www.youtube.com/embed/6xJ33UD-NfE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>



## QR-код

<div style="display: inline-block; vertical-align: top;">
<img src="https://lh3.googleusercontent.com/0FGhp05Cek_U63x4aRgD0DKrcoNSwvonOtrR16Zk3ol-8Owg3ir8DKw3bts7PU5V8x5kdCoL4466qY6BYLvZNuLpDYsnwZKu_NslfenqrjSVfqQZ68DXzKiYRbXKF3ALhHGjFWyb9SuwrYU3VDzdFJUsCAF_80I95Bo6e2eUxmxfWUZQt5Wxp7ilyg0Q2LzisLVsgj82tzR5TYaQ2kPRPqpoBb4hzleGs3p5yt4PoCS00QiMIjcMqcgejaML8musmMJ3Kh6zCThLzgkzv8r2LFFjotQadaZ_BkSDe6h-YaU_10bXD-Med-BFzwKW4NRYRMBBhthSVy0yarC0jZcbz55rlN8IhHio_729xUH7FXx--XNNtJ08DKDmE0lVnsnZmWq6o1JEYTthbG8Lo5I3O2pyzkUwBQwSkKUnhksrkQ32BiFInpgT2tEvGILCxjrOdsWRXK81s2KqoUMWWZ7o1DUPxQIBA-v2O-oPxh8Ao-9fPck_ohxWw_OgjopUNh4WtFfhoy26MXq0dCPs2ItcI2jS54W5CgV41tEQyHkf2TUIZRc_AE5b_zZGYLKnx3H0cfOqLj0qwyKpvEKxCteM7cyPNoQaNX2tg_50wJdZe9A3cN6EN5cIud8Ou851RotAFO5miHFyGzpZXNyIGe5_1onyctZ6vaLb1PRrwUbBoXHzjlv7PY7X9e93A4i5n-9-Hpsy9Q=w1920-h937-ft" width="400" >
<p><i>Момент распознавания QR-кода во время зачетной попытки</i></p>
</div>
<div style="display: inline-block; vertical-align: top;padding-left:20px; width:50%">
<p>
Для выполнения задачи распознавания QR-кода была использована библиотека PyZbar. 
С целью повышения результативности и точности распознавания QR-кода полеты производились на небольшой высоте по точкам, расположенным вокруг данного объекта.
</p>
</div>

## Посадка 

Посадка выполняется в 3 этапа:
1. Перелет к предполагаемой зоне посадки и зависание на высоте 1.5м
2. Спуск до высоты в 0.85м с 3 корректировками по координатам маркера относительно aruco_map
3. Спуск в течение нескольких секунд с постоянной корректировкой по координатам маркера посадки в системе координат body (так как aruco маркеры могут быть уже не видны), вместо navigate используется set_position


<iframe width="560" height="315" src="https://www.youtube.com/embed/8nVGoWkdYcA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Gazebo
По причине отсутствия возможности тестирования кода на своем реальном дроне было принято решение воспользоваться симулятором Gazebo.

Для запуска пакета ПО Клевера в симуляторе можно использовать [набор скриптов](https://github.com/vas0x59/clever_sim) или [оригинальную инструкцию от PX4](https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html).

Для Innopolis Open было создано несколько тестовых сцен. [ior2020_uav_L22_AERO_sim](https://github.com/vas0x59/ior2020_uav_L22_AERO_sim)

Также использование симулятора ускорило отладку полного выполнения кода, так как запуск производился с real time factor=2.5

<img src="https://github.com/vas0x59/ior2020_uav_L22_AERO_info/raw/master/to_Gitbook/content/real_map.jpg" height="250">
<img src="https://github.com/vas0x59/ior2020_uav_L22_AERO_info/raw/master/to_Gitbook/content/big_map.jpg" height="250">

При тестировании выявлены некоторые проблемы (некорректное положение aruco_map) с использованием дисторсии в плагине камеры, поэтому в симуляторе использовалась камера типа Pinhole (без искажений от объектива)



## ROS
Созданные ноды, топики, сообщения и сервисы


### Nodes

<ul style="list-style: none; font-size:15px; padding-left:10px">
 <li>l22_aero_vision/<b>color_r_c.py</b> - распознавание цветных объектов</li>
 <li>l22_aero_vision/<b>viz.py</b> - визуализация в RViz</li>
 <li>l22_aero_code/<b>full_task.py</b> - основной код  </li>
</ul>

### Topics

<ul style="list-style: none; font-size:15px; padding-left:10px">
 <li><b>/l22_aero_color/markers</b> l22_aero_vision/ColorMarkerArray - список прямоугольных маркеров</li>
 <li><b>/l22_aero_color/circles</b> l22_aero_vision/ColorMarkerArray - список круглых маркеров</li>
 <li><b>/l22_aero_color/debug_img</b> sensor_msgs/Image - изображение для отладки </li>
 <li><b>/qr_debug</b> sensor_msgs/Image - изображение для отладки </li>
</ul>


### Messages
#### ColorMarker
```cpp
string color
int16 cx_img
int16 cy_img
float32 cx_cam
float32 cy_cam
float32 cz_cam
float32 size1
float32 size2
int16 type
```
#### ColorMarkerArray
```cpp
std_msgs/Header header
l22_aero_vision/ColorMarker[] markers
```
### Services
#### SetParameters
```
float32 rect_s1
float32 rect_s2
float32 circle_r
int32 obj_s_th
int32 offset_w
int32 offset_h
---
```

