# Подготовка образа для RPI

## Установка образа операционной системы на MicroSD карту
![Etcher](https://etcher.io/static/screenshot.gif)

Для установки образа рекомендуем воспользоваться утилитой Etcher [[скачать]](https://etcher.io/)

## Образы MicroSD
Ниже представлен список настроенных образов для установки на SD карту RPI.

### First release 13092017
Скачать [ [2GB со сжатием (.zip)](https://drive.google.com/open?id=1_z3pf1BshIutqmmhkMpuGdNS_8fcxNlH) / [7.4GB без сжатия (.img)](https://drive.google.com/open?id=0B3h5BTtcX6gwMWtnc3J6eHIyb2M) ]
* Настроена сеть 192.168.11.1, настроен wifi (CLEVER-XXXX)
* Настроен ROS и локальное окружение
* Установлен marker_navigator
* Сконфигурирован UART, камера
* Добавлен скрипт clever_setup для сброса образа в исходное состояние

### ROS [[скачать]](https://drive.google.com/open?id=0B3h5BTtcX6gwX1hXX21MZ1dZOUE)
* Доустановлен ROS

### NETWORK [[скачать]](https://drive.google.com/open?id=0B3h5BTtcX6gwMVYzU2tRckNoQjQ)
* Взят Raspbian Stretch
* Настроена сеть 192.168.11.1 и DHCP-сервер
