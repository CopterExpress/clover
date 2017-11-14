Управление коптером с Arudino
===

Для взаимодействия с ROS-топиками и сервисами на Raspberry Pi можно использовать библиотеку [rosserial_arduino](http://wiki.ros.org/rosserial_arduino).

Основной туториал: http://wiki.ros.org/rosserial_arduino/Tutorials

Arudino необходимо установить на Клевер и подключить по USB-порту.

Настройка Aruino IDE
---

* Установить внешнюю библиотеку `rosserial`: Скетч -> Подключить библиотеку -> Управлять библиотеками...
* Ввести rosserial и нажать "Установка"

![](/assets/rosserial.png).

* Скачать и скопировать библиотеку ROS-сообщений Клевера в `<папка скетчей>/libraries`.