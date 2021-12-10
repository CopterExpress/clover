# Сборка на собственной машине

Настройка среды для симуляции с нуля требует некоторых усилий, однако это приведет к улучшению производительности и к уменьшению вероятности появления проблем с драйверами.

> **Hint** Смотрите актуальный набор команд установки необходимого ПО для запуска симулятора Клевера в скрипте сборки виртуальной машины с симулятором: [`install_software.sh`](https://github.com/CopterExpress/clover_vm/blob/master/scripts/install_software.sh).

Требования для сборки: **Ubuntu 20.04**.

## Установка ROS

Установите ROS Noetic используя [официальную документацию по установке](http://wiki.ros.org/noetic/Installation/Ubuntu) (Desktop или Full установка).

Добавьте выполнение инициализирующего скрипта ROS `setup.bash` в ваш файл `.bashrc`:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Установите необходимые инструменты, которые понадобятся для дальнейшей установки:

```bash
sudo apt install build-essential git python3-pip python3-rosdep
```

## Создание рабочего пространства для симулятора

Создайте рабочее пространство:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Склонируйте исходный код пакетов Clover:

```bash
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/CopterExpress/clover
git clone --depth 1 https://github.com/CopterExpress/ros_led
git clone --depth 1 https://github.com/ethz-asl/mav_comm
```

Установите все зависимости, используя `rosdep`:

```bash
cd ~/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Установите Python-зависимости:

```bash
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt
```

## Загрузка исходного кода PX4

Сборка PX4 будет осуществлена вместе с другими пакетами в нашем рабочем пространстве. Вы можете загрузить его прямо в рабочее пространство или поместить куда-нибудь и создать симлинк к `~/catkin_ws/src`. Нам также нужно будет поместить его подмодули `sitl_gazebo` и `mavlink` в `~/catkin_ws/src`.

Склонируйте исходный код PX4 и создайте необходимые симлинки:

```bash
git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
ln -s ~/PX4-Autopilot ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/mavlink ~/catkin_ws/src/
```

> **Hint** Вы можете использовать более позднюю версию PX4 с большим риском, что что-то не заработает.

## Установка зависимостей PX4

PX4 имеет свой собственный скрипт для установки зависимостей. Воспользуемся им:

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh
```

Он установит все, что нужно для сборки PX4 и SITL.

> **Hint** Также вы можете пропустить установку ARM тулчейна, если вы не планируете компилировать PX4 для вашего полетного контроллера. Для этого воспользуйтесь флагом `--no-nuttx`: `sudo ./ubuntu.sh --no-nuttx`.

Установите дополнительные необходимые Python-пакеты:

```bash
pip3 install --user toml
```

## Добавление рамы Клевера

Добавьте в PX4 раму Клевера с помощью следующей команды:

```bash
ln -s ~/catkin_ws/src/clover/clover_simulation/airframes/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
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

> **Note** Если процесс сборки завершится с ошибкой, связанной с недостатком памяти (`c++: fatal error: Killed signal terminated program cc1plus`), уменьшите количество параллельно исполняемых процессов используя ключ `-j`. Например, чтобы использовать только два параллельных процесса используйте команду `catkin_make -j2`.

## Запуск симулятора

Чтобы удостовериться в том, что все было собрано корректно, попробуйте запустить симулятор:

```bash
roslaunch clover_simulation simulator.launch
```

Вы можете проверить автономный полет используя скрипты в директории `~/catkin_ws/src/clover/clover/examples`.

## Дополнительные шаги

Опционально вы можете установить systemd-сервис для roscore для того, чтобы roscore был постоянно запущен в фоне:

```bash
sed -i "s/pi/$USER/g" ~/catkin_ws/src/clover/builder/assets/roscore.service
sudo cp ~/catkin_ws/src/clover/builder/assets/roscore.service /etc/systemd/system
sudo systemctl enable roscore
sudo systemctl start roscore
```

Установите любой веб-сервер, чтобы раздавать веб-инструменты Клевера (директория `~/.ros/www`), например, Monkey:

```bash
wget https://github.com/CopterExpress/clover_vm/raw/master/assets/packages/monkey_1.6.9-1_amd64.deb -O /tmp/monkey_1.6.9-1_amd64.deb
sudo apt-get install -y /tmp/monkey_1.6.9-1_amd64.deb
sed "s/pi/$USER/g" ~/catkin_ws/src/clover/builder/assets/monkey | sudo tee /etc/monkey/sites/default
sudo -E sh -c "sed -i 's/SymLink Off/SymLink On/' /etc/monkey/monkey.conf"
sudo cp ~/catkin_ws/src/clover/builder/assets/monkey.service /etc/systemd/system/monkey.service
sudo systemctl enable monkey
sudo systemctl start monkey
```
