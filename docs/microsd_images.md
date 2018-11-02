# Образ для Raspberry Pi

На образе установлены:

* Raspbian Stretch
* ROS Kinetic
* [Пакет ПО для Клевера](https://github.com/CopterExpress/clever)

**Свежую версию образа можно [скачать на GitHub в разделе Releases](https://github.com/CopterExpress/clever/releases).**

## Установка образа ОС на MicroSD карту

Для установки образа воспользуйтесь утилитой [Etcher](https://etcher.io).

[![Etcher](assets/etcher.gif)](https://etcher.io)

## Версия образа

Версию установленного образа можно узнать в файле `/etc/clever_version`:

```bash
cat /etc/clever_version
```
