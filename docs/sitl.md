Симуляция PX4
===

Симуляция PX4 возможна в ОС Linux и macOS с использованием систем симуляции физической среды [jMavSim](https://pixhawk.org/dev/hil/jmavsim) и [Gazebo](http://gazebosim.org).

jMavSim является легковесной средой, предназначенной только для тестирование мультироторных летательных систем; Gazebo – универсальная среда для любых типов роботов.

Запуск PX4 Sitl
--

1. Склонировать репозиторий с PX4.

```bash
git clone https://github.com/PX4/Firmware.git
cd Firmware
```

jMavSim
--

Основная статья: https://dev.px4.io/en/simulation/jmavsim.html

Для симуляции с использованием легковесной среды jMavSim используйте команду:

```bash
make posix_sitl_default jmavsim
```

Для использования модуля рассчета позиции LPE вместо EKF2, используйте:

```bash
make posix_sitl_lpe jmavsim
```

Gazebo
--

Основная статья: https://dev.px4.io/en/simulation/gazebo.html

Для начала установите Gazebo 7. На Mac:

```bash
brew install gazebo7
```

На Linux (Debian):

```bash
sudo apt-get install gazebo7 libgazebo7-dev
```

Запустите симуляцию, находясь в папке Firmware:

```bash
make posix_sitl_default gazebo
```

Можно запустить симуляцию в headless режиме (без оконного клиента). Для этого используйте команду:

```bash
HEADLESS=1 make posix_sitl_default gazebo
```
