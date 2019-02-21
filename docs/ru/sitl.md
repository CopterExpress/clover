# Симуляция PX4

Основная статья: https://dev.px4.io/en/simulation/

Симуляция PX4 возможна в ОС Linux и macOS с использованием систем симуляции физической среды [jMavSim](https://pixhawk.org/dev/hil/jmavsim) и [Gazebo](http://gazebosim.org).

jMavSim является легковесной средой, предназначенной только для тестирование мультироторных летательных систем; Gazebo – универсальная среда для любых типов роботов.

## Запуск PX4 Sitl

1. Склонировать репозиторий с PX4.

```bash
git clone https://github.com/PX4/Firmware.git
cd Firmware
```

## jMavSim

Основная статья: https://dev.px4.io/en/simulation/jmavsim.html

Для симуляции с использованием легковесной среды jMavSim используйте команду:

```bash
make posix_sitl_default jmavsim
```

Для использования модуля расчета позиции LPE вместо EKF2, используйте:

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

## Подключение

QGroundControl автоматически подключится к запущенной симуляции при запуске. Работа будет осуществляться также, как и с настоящим полетным контроллером.

Для подключение MAVROS к симуляции необходимо использовать протокол UDP, локальный IP-адрес и порт 14557, например:

```bash
roslaunch mavros px4.launch fcu_url:=udp://@127.0.0.1:14557
```

## Как завести SITL самому на чистой системе Ubuntu

Для того, что бы запустить симулятор на чистом образе Ubuntu вам потребуется склонировать основной репозиторий PX4, установить ROS Kinetic и установить все сопутствующие модули.
Выполните команды для обновления списка пакетов и установите `git` в случае его отсутствия.

```bash
sudo apt-get update
sudo apt-get install git
```

Склонируйте себе основной репозиторий `PX4` и при необходимости верните его до стабильной версии v1.8.2:

```bash
git clone https://github.com/PX4/Firmware.git
git checkout v1.8.2
```

### Установка ROS Kinetic

Настройте систему для скачивания информации с `packages.ros.org`.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Настройте ключ подлючения:

```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Обновите списки пакетов и установите полную версию `ROS Kinetic`.

```bash
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

Инициализируйте и обновите `rosdep`.

```bash
sudo rosdep init
rosdep update
```

### Установка необходимых модулей

Для коректной работы Gazebo необходимо поставить соответствующие пакеты, (их список можно подсмотреть в [контейнерах для сборки](https://github.com/PX4/containers/blob/master/docker/px4-dev/Dockerfile_base) и [запуска в симуляторе](https://github.com/PX4/containers/blob/master/docker/px4-dev/Dockerfile_simulation))

```bash
apt-get update && apt-get -y --quiet --no-install-recommends install \
		bzip2 \
		ca-certificates \
		ccache \
		cmake \
		cppcheck \
		curl \
		dirmngr \
		doxygen \
		file \
		g++ \
		gcc \
		gdb \
		git \
		gnupg \
		gosu \
		lcov \
		libfreetype6-dev \
		libgtest-dev \
		libpng-dev \
		lsb-release \
		make \
		ninja-build \
		openjdk-8-jdk \
		openjdk-8-jre \
		openssh-client \
		pkg-config \
		python-pip \
		python-pygments \
		python-setuptools \
		rsync \
		shellcheck \
		tzdata \
		unzip \
		wget \
		xsltproc \
		zip \
        ant \
		gazebo9 \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		libeigen3-dev \
		libgazebo9-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		pkg-config \
		protobuf-compiler
```

``` bash
python -m pip install --upgrade pip \
	&& pip install setuptools pkgconfig wheel \
	&& pip install argparse argcomplete coverage jinja2 empy numpy requests serial toml pyyaml cerberus
```
