# Настройка расположения основной камеры

Расположение и ориентация основной камеры задается в файле `~/catkin_ws/src/clever/clever/launch/main_camera.launch`:

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0 0 -0.07 -1.5707963 0 3.1415926 fcu main_camera_optical"/>
```

Эта строка задает статическую трансформацию между фреймом `fcu` ([соответствует корпусу полетного контроллера](/docs/frames.md)) и камерой (`main_camera_optical`) в формате:

```
сдвиг_x сдвиг_y сдвиг_z угол_рысканье угол_тангаж угол_крен
```

Фрейм камеры задается таким образом, что:
* x указывает направо на изображении;
* y указывает вниз на изображении;
* z указывает от плоскости матрицы камеры.

Сдвиги задаются в метрах, углы задаются в радианах. Корректность установленной трансформации может быть проверена с использованием [rviz](/docs/rviz.md).

## Настройки для Клевера

### Клевер 3, камера вниз

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 1.5707963 0 3.1415926 fcu main_camera_optical"/>
```

### Клевер 3, камера вверх

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 0.07 -1.5707963 0 0 fcu main_camera_optical"/>
```

### Клевер 2, камера вниз

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0 0 -0.07 -1.5707963 0 3.1415926 fcu main_camera_optical"/>
```
