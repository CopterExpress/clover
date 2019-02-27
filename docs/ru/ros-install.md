# Установка и настройка пакета ROS Kinetic

Для работы с такими инструментами как: `rqt`, `rviz` и т.д., а также для запуска симулятора `SITL`. Вам потребуется установленный и настроенный пакет `ROS`.

> **Hint** Для более полной инструкции по установке смотрите [основную статью](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## Установка ROS Kinetic на Ubuntu

Для того, что бы загрузить и установить правильную версию пакета требуется сделать настройки репозиториев, для этого откройте "Программы и обновления" и разрешите `restricted`, `universe` и `multiverse`.

Настройте свою систему, для того что бы вы могли принимать программное обеспечение с `packages.ros.org`, выполнив команду:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Настройте ключи доступа в своей системе для правильной загрузки.

```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Убедитесь в том, что вы имеете последние версии индексов пакетов.

```bash
sudo apt-get update
```

Теперь установите сам пакет `ROS`.

+ Если вы планируете использовать `ROS` вместе с симуляцией(так же содержит инструменты: `rqt`, `rviz` и т.д).

```bash
sudo apt-get install ros-kinetic-desktop-full
```

+ Если вы планируете использовать `ROS` исключительно работать с инструментами: `rqt`, `rviz` и т.д.

```bash
sudo apt-get install ros-kinetic-desktop
```

После установки пакета, вам нужно инициализировать `rosdep`.
`rosdep` позволит вам легко устанавливать системные зависимости для источника, который вы хотите скомпилировать и необходим для запуска некоторых основных компонентов в ROS.

```bash
sudo rosdep init
rosdep update
```

Что бы каждый раз в ручном режиме на запускать ваш `ROS`, вы можете настроить среду так, что бы он загружался автоматически на каждой сессии.

```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Если вы хотите установить какие-либо дополнительные пакеты для вашего `ROS Kinetic` просто используйте.

```bash
sudo apt-get install ros-kinetic-PACKAGE
```
