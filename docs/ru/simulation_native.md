# Сборка на собственной машине

Настройка среды для симуляции с нуля требует некоторых усилий, однако это приведет к улучшению производительности и к уменьшению вероятности появления проблем с драйверами.

> **Hint** Смотрите актуальный набор команд установки необходимого ПО для запуска симулятора Клевера в скрипте сборки виртуальной машины с симулятором: [`install_software.sh`](https://github.com/CopterExpress/clover_vm/blob/master/scripts/install_software.sh).

Требования для сборки: Ubuntu 20.04 и [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Создание рабочего пространства для симулятора

В этой статье мы будем использовать `catkin_ws` как имя рабочего пространства (вы можете поменять её). Мы создадим её в домашнем каталоге текущего пользователя (`~`).

Создайте рабочее пространство и загрузите исходный код Клевера:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/CopterExpress/clover
git clone --depth 1 https://github.com/CopterExpress/ros_led
git clone --depth 1 https://github.com/ethz-asl/mav_comm
```

Установите все зависимости, используя `rosdep`:

```bash
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Установите Python-зависимости:

```bash
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt
```

## Загрузка исходного кода PX4

Сборка PX4 будет осуществлена вместе с другими пакетами в нашем рабочем пространстве. Вы можете загрузить его прямо в рабочее пространство или поместить куда-нибудь и создать симлинк к `~/catkin_ws/src`. Нам также нужно будет поместить его подмодуль `sitl_gazebo` в `~/catkin_ws/src`. Для упрощения мы загрузим прошивку прямо в рабочее пространство:

```bash
cd ~/catkin_ws/src
git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
ln -s ~/PX4-Autopilot ~/catkin_ws/src/PX4-Autopilot
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/sitl_gazebo
```

## Установка зависимостей PX4

PX4 имеет свой собственный скрипт для установки зависимостей. Воспользуемся им:

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh
```

Он установит все, что нужно для сборки PX4 и SITL.

Также вы можете пропустить установку ARM тулчейна, если вы не планируете компилировать PX4 для вашего полетного контроллера. Для этого воспользуйтесь флагом `--no-nuttx`:

```
sudo ./ubuntu.sh --no-nuttx
```

## Добавление рамы Клевера

Добавьте в PX4 раму Клевера с помощью следующей команды:

```bash
ln -s "$(catkin_find clover_simulation airframes)"/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

## Установка датасетов geographiclib

Для `mavros` нужны датасеты geographiclib:

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

## Сборка симулятора

После установки всех зависимостей можно начинать сборку рабочего пространства:

```bash
cd ~/catkin_ws
catkin_make
```

> **Note** Некоторые файлы, особенно плагины Gazebo, требуют большого объема оперативной памяти для сборки. Вы можете уменьшить количество параллельных процессов; количество параллельных процессов должно быть равно объёму RAM в гигабайтах, поделенному на 2. Например, для машины с 16Гб следует указывать не более 8 процессов. Вы можете указать количество процессов, используя флаг `-j` : ```catkin_make -j8```

## Запуск симулятора

Чтобы удостовериться в том, что все было собрано корректно, попробуйте запустить симулятор:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch clover_simulation simulator.launch
```
