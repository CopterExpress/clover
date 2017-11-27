# Clever

Пакет программ и библиотек для Клевера.

Основная документация
---------------------

https://copterexpress.gitbooks.io/clever/content/

Установка
---------

Склонировать репозиторий в папку `/home/pi/catkin_ws/src/clever` (**важно**):

```bash
cd ~/catkin_ws/src
git clone https://github.com/CopterExpress/clever_bundle.git clever
```

Пересобрать ROS-пакеты:

```bash
cd ~/catkin_ws
catkin_make -j1
```

Включить сервис roscore (если он не включен):

```bash
sudo systemctl enable /home/pi/catkin_ws/src/clever/deploy/roscore.service
sudo systemctl start roscore
```

Включить сервис clever:

```bash
sudo systemctl enable /home/pi/catkin_ws/src/clever/deploy/clever.service
sudo systemctl start clever
```

Зависимости
-----------

Необходимые для работы ROS-пакеты:

* `mavros`
* `rosbridge_suite`
* `web_video_server`
* `cv_camera`
* `nodelet`
* `dynamic_reconfigure`
* `bondcpp`, ветка `master`
* `roslint`
* `rosserial`

TODO: внести в package.xml
