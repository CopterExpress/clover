# Настройка расположения основной камеры

Расположение и ориентация основной камеры задается в файле `~/catkin_ws/src/clever/clever/launch/main_camera.launch` в строке:

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0 0 -0.07 -1.5707963 0 3.1415926 fcu bottom_camera_optical"/>
```

Эта строка задает статическую трансформацию между фреймом fcu (соответствует корпусу полетного контроллера) и камерой в формате:

````
сдвиг_x сдвиг_y сдвиг_z угол_рысканье угол_тангаж угол_крен
```

Сдвиги задаются в метрах, углы задаются в радианах.

## Настройки для Клевера

### Клевер 3, камера вниз

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 1.5707963 0 3.1415926 fcu bottom_camera_optical"/>
```

### Клевер 3, камера вверх

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 0.07 -1.5707963 0 0 fcu bottom_camera_optical"/>
```