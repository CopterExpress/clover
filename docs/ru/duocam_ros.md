# Трансляция изображение с DuoCam в ROS-топик

Для задач связанных с машинным зрением удобно работать с изображением с камер через ROS-топики.
DuoCam включает в себя две камеры: визуальная камера VEYE-MIPI-327E и тепловизор HT-201. В этой статье рассмотрим как настроить получение с них ображения и вывод его в ROS-топики.



## VEYE-MIPI-327E driver:
Основаная статья: http://wiki.veye.cc/index.php/V4L2_mode_for_Raspberry_Pi

Установка драйвера:
```
git clone https://github.com/veyeimaging/raspberrypi_v4l2.git
cd raspberrypi_v4l2/release/
chmod +x *
sudo ./install_driver.sh veye327

Check and Test the Camera:
dmesg | grep veye
ls /dev/video0
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext
```






## Seek Thermal CompactPRO driver:
Установка драйвера тепловизора HT-201:
```
sudo apt install cmake libopencv-dev libusb-1.0-0-dev v4l2loopback-utils
git clone https://github.com/OpenThermal/libseek-thermal.git
cd libseek-thermal
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig

sudo vim /etc/udev/rules.d/seekpro.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="289d", ATTRS{idProduct}=="0011", MODE="0666", GROUP="users"
sudo udevadm control --reload-rules && sudo udevadm trigger

seek_test_pro
seek_create_flat_field -tseekpro
seek_viewer --camtype=seekpro --colormap=2 --rotate=0 -F flat_field.png

sudo modprobe v4l2loopback devices=1
seek_viewer --camtype=seekpro --colormap=2 --rotate=0 --mode=v4l2 --output=/dev/video2 &

v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext
v4l2-ctl -d /dev/video0 -V
v4l2-ctl -d /dev/video1 -V --info --list-formats --list-formats-ext
```






## Передняя USB-камера в ROS

vim catkin_ws/src/clover/clover/launch/front_camera.launch
```
<launch>

    <arg name="direction_z" default="forward"/> <!-- direction the camera points: forward, backward -->
    <arg name="device2" default="/dev/video1"/> <!-- v4l2 device -->
    <arg name="simulator" default="false"/>

    <node if="$(eval direction_z == 'forward')" pkg="tf2_ros" type="static_transform_publisher" name="front_camera_frame" args="0.03 0 0.05 -1.5707963 0 -1.5707963 base_link front_camera_optical"/>
    <node if="$(eval direction_z == 'backward')" pkg="tf2_ros" type="static_transform_publisher" name="front_camera_frame" args="-0.03 0 0.05 1.5707963 0 -1.5707963 base_link front_camera_optical"/>

    <!-- Template for custom camera orientation -->
    <!-- Camera position and orientation are represented by base_link -> main_camera_optical transform -->
    <!-- static_transform_publisher arguments: x y z yaw pitch roll frame_id child_frame_id -->
    <!-- <node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 -1.5707963 0 3.1415926 base_link main_camera_optical"/> -->

    <!-- camera nodelet manager -->
    <node pkg="nodelet" type="nodelet" name="front_camera_nodelet_manager" args="manager" output="screen" clear_params="true" respawn="true">
        <param name="num_worker_threads" value="2"/>
    </node>

    <!-- camera node -->
    <node pkg="nodelet" type="nodelet" name="front_camera" args="load cv_camera/CvCameraNodelet front_camera_nodelet_manager" launch-prefix="rosrun clover waitfile $(arg device2)" clear_params="true" unless="$(arg simulator)" respawn="true">
        <param name="device_path" value="$(arg device2)"/>
        <param name="frame_id" value="front_camera_optical"/>

        <param name="rate" value="100"/> <!-- poll rate -->
        <param name="cv_cap_prop_fps" value="40"/> <!-- camera FPS -->
        <param name="capture_delay" value="0.02"/> <!-- approximate delay on frame retrieving -->

        <!-- camera resolution -->
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
    </node>

</launch>
```
