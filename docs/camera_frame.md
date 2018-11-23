# Настройка расположения основной камеры

Расположение и ориентация основной камеры задается в файле `~/catkin_ws/src/clever/clever/launch/main_camera.launch`:

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0 0 -0.07 -1.5707963 0 3.1415926 fcu main_camera_optical"/>
```

Эта строка задает статическую трансформацию между фреймом `fcu` ([соответствует корпусу полетного контроллера](frames.md)) и камерой (`main_camera_optical`) в формате:

```txt
сдвиг_x сдвиг_y сдвиг_z угол_рысканье угол_тангаж угол_крен
```

Фрейм камеры задается таким образом, что:

* **<font color=red>x</font>** указывает направо на изображении;
* **<font color=green>y</font>** указывает вниз на изображении;
* **<font color=blue>z</font>** указывает от плоскости матрицы камеры.

Сдвиги задаются в метрах, углы задаются в радианах. Корректность установленной трансформации может быть проверена с использованием [rviz](rviz.md).

## Настройки для Клевера

### 1. Камера направлена вниз, шлейф назад

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 -1.5707963 0 3.1415926 fcu main_camera_optical"/>
```

![](assets/camera_option_1.png)

### 2. Камера направлена вниз, шлейф вперёд

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 1.5707963 0 3.1415926 fcu main_camera_optical"/>
```

![](assets/camera_option_2.png)

### 3. Камера направлена вверх, шлейф назад

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 0.07 1.5707963 0 0 fcu main_camera_optical"/>
```

![](assets/camera_option_3.png)

### 4. Камера направлена вверх, шлейф вперёд

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 0.07 -1.5707963 0 0 fcu main_camera_optical"/>
```

![](assets/camera_option_4.png)
