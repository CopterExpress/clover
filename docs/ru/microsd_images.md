# Образ для Raspberry Pi

На образе установлены:

* [Raspbian](https://www.raspberrypi.org/downloads/raspbian/) Stretch
* [ROS Kinetic](http://wiki.ros.org/kinetic)
* [Пакет ПО для Клевера](https://github.com/CopterExpress/clever)

## Скачать

Последний стабильный релиз:

**<a id="download-latest-release"></a>**

<script type="text/javascript">
    // get latest release from GitHub
    fetch('https://api.github.com/repos/CopterExpress/clever/releases').then(function(res) {
        return res.json();
    }).then(function(data) {
        // look for stable release
        let stable;
        for (let release of data) {
            if (!release.prerelease && !release.draft) {
                stable = release;
                break;
            }
        }
        let el = document.querySelector('#download-latest-release');
        let name = stable.name;
        let link = stable.assets[0].browser_download_url;
        el.innerHTML = name + ' – скачать'
        el.href = link;
    });
</script>

Свежую версию образа можно [скачать на GitHub в разделе Releases](https://github.com/CopterExpress/clever/releases).

> **Hint** Стабильной и поддерживаемой версией образа является релиз, помеченный плашкой **Latest release**.

<img src="../assets/image.png" width=400 alt="Скачивание образа">

## Запись образа ОС на MicroSD карту

Для установки образа вы можете воспользоваться утилитой [Etcher](https://etcher.io).

[![Etcher](../assets/etcher.gif)](https://etcher.io)

После записи образа на SD-карту, вы можете подключаться к [Клеверу по Wi-Fi](wifi.md), получать [доступ по SSH](ssh.md) и использовать остальные функции.

## Версия образа

Версию установленного образа можно узнать в файле `/etc/clever_version`:

```bash
cat /etc/clever_version
```

## Обновление и пересборка пакетов Клевера на Raspberry

> **Warning** Предрелизные пакеты Клевера могут содержать ошибки!

Перед выполнением дальнейших действий надо [подключить Raspberry к интернету](network.md).

Обновление исходного кода пакетов Клевера осуществляется с помощью системы контроля версий [Git](https://git-scm.com). Локальный репозиторий с исходным кодом находится в директории `/home/pi/catkin_ws/src/clever`. Для работы с этим репозиторием в первый раз надо выполнить команду:

```bash
cd /home/pi/catkin_ws/src/clever
git fetch --unshallow
```

Для обновления исходного кода пакетов выполните:

```bash
cd /home/pi/catkin_ws/src/clever
git checkout master
git pull origin master
```

Для использования обновлённых пакетов их требуется собрать. Перед сборкой следует убедиться в том, что все зависимости пакетов Клевера установлены. Для этого выполните команду:

```bash
sudo apt update
rosdep update
cd /home/pi/catkin_ws
rosdep install --from-paths src --ignore-src -y
```

Пересборка пакетов - потенциально доглий и ресурсоёмкий процесс. Перед его началом следует остановить `clever.service`, производить сборку рекомендуется в 1 поток - иначе может закончиться оперативная память. Пересборку можно инициировать следующими командами:

```bash
sudo sysmemctl stop clever
cd /home/pi/catkin_ws
catkin_make -j1
```
