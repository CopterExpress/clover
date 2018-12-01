# Образ для Raspberry Pi

На образе установлены:

* [Raspbian](https://www.raspberrypi.org/downloads/raspbian/) Stretch
* [ROS Kinetic](http://wiki.ros.org/kinetic)
* [Пакет ПО для Клевера](https://github.com/CopterExpress/clever)

**Свежую версию образа можно [скачать на GitHub в разделе Releases](https://github.com/CopterExpress/clever/releases).**

> **Hint** Стабильной и поддерживаемой версией образа является релиз, помеченный плашкой **Latest release**.

<img src="../assets/image.png" width=400 alt="Скачивание образа">

## Установка образа ОС на MicroSD карту

Для установки образа вы можете воспользоваться утилитой [Etcher](https://etcher.io).

[![Etcher](../assets/etcher.gif)](https://etcher.io)

## Версия образа

Версию установленного образа можно узнать в файле `/etc/clever_version`:

```bash
cat /etc/clever_version
```
