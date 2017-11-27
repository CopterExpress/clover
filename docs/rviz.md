Использование rviz
===

Инструмент [rviz](http://wiki.ros.org/rviz) позволяет в реальном времени визуализировать на 3D-сцене все компоненты роботехнической системы — системы координат, движущиеся части, показания датчиков, изображения с камер.

Для использования этого инструмента необходим компьютер с ОС Ubuntu Linux (либо виртуальная машина, например [Parallels Desktop Lite](https://itunes.apple.com/ru/app/parallels-desktop-lite/id1085114709?mt=12) или [VirtualBox](https://www.virtualbox.org)).

На него необходимо установить пакет `ros-kinetic-desktop-full` или `ros-kinetic-desktop`, используя [документацию по установке](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Запуск rviz
---

Для запуска визуализация состояния Клевера в реальном времени, необходимо подключиться к нему по Wi-Fi (`CLEVER-xxx`) и запустить rviz, указав соответствующий ROS_MASTER_URL:

```bash
ROS_MASTER_URI=http://192.168.11.1:11311 rviz
```

Если соединение не устанавливается, необходимо убедиться, что в `.bashrc` Клевера присутствует строка:

```bash
export ROS_IP=192.168.11.1
```
