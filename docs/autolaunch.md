Автозапускаемое ПО
===

systemd
---

Основная документация: https://wiki.archlinux.org/index.php/Systemd_(Русский).

Все автоматически стартуемое ПО Клевера запускается в виде systemd-сервиса `clever.service`.

Сервис может быть перезапущен командой:

```bash
sudo systemctl restart clever
```

Текстовый вывод ПО можно просмотреть с помощью команды journalctl:

```bash
journalctl -u clever
```

Для того, запустить ПО Клевера непосредственно в текущей консольной сессии, вы можете использовать `roslaunch`:

```bash
sudo systemctl stop clever
roslaunch clever clever.launch
```

Вы можете выключить автозапуск ПО Клевера с помощью команды `disable`:

```bash
sudo systemctl disable clever
```

roslaunch
---

Основная документация: http://wiki.ros.org/roslaunch.

Список объявленных для запуска нод / программ указывается в файле `/home/pi/catkin_ws/src/clever/clever/launch/clever.launch`.

Вы можете добавить собственную ноду в список автозапускаемых. Для этого разместите ваш запускаемый файл (например, `my_program.py`) в каталог `/home/pi/catkin_ws/src/clever/clever/src`. Затем добавьте запуск вашей ноды в `clever.launch`, например:

```xml
<node name="my_program" pkg="clever" type="my_program.py" output="screen"/>
```

Запускаемый файл должен иметь *permission* на запуск:

```bash
chmod +x my_program.py
```

При использовании скриптовых языков вначале файла должен стоять [shebang](https://ru.wikipedia.org/wiki/Шебанг_(Unix)), например:

```bash
#!/usr/bin/env python
```
