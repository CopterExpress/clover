Пакет программ на Raspberry Pi
===

Пакет программ clever_bundle, устанавливающийся на Raspberry Pi, позволяет:

* [Настраивать и управлять коптером используя QGroundControl с соединением по Wi-Fi](/docs/gcs_bridge.md)
* [Использовать веб-пульт управления квадрокоптером](/docs/web_rc.md)
* [Получать доступ к Raspberry Pi при помощи SSH](/docs/ssh.md)
* Анализировать полеты квадрокоптера с помощью RViz и RosBag
* [Работать с камерой для CV](/docs/camera.md)
* Работать с камерой для FPV
* [Управлять полетом коптера программно, используя модуль offboard](/docs/offboard.md)
* [Осуществлять навигацию в поле ArUco-маркеров](/docs/aruco.md)
* [Использовать внешний 3G-модем для осуществление связи коптера с Интернетом](/docs/3g.md)
* Разрабатывать произвольные модули и системы

Установка clever_bundle
---

Для установка пакета clever_bundle необходимо скачать последнюю версию образа SD-карты и загрузить его на флешку, например, используя программу [Etcher](https://etcher.io).

Информация
---

Образ SD-карты включает в себя:

* ОС [Raspbian Jessie](https://www.raspberrypi.org/downloads/raspbian/)
* Фреймворк [ROS](/docs/ros.md)
* Пакет [MAVROS](/docs/mavros.md) для связи с PixHawk по MAVLINK
* Дополнительные пакеты ROS: web_video_server, usb_cam, rosbridge_suite и другие
* Пакет программ clever_bundle
