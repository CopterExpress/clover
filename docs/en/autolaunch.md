Software autorun
===

systemd
---

Main documentation: [https://wiki.archlinux.org/index.php/Systemd_(Russian)](https://wiki.archlinux.org/index.php/Systemd_(Russian)).

All automatically started Clover software is launched as a `clever.service` systemd service.

The service may be restarted by the `systemctl` command:

```(bash)
sudo systemctl restart clever
```

Text output of the software can be viewed using the `journalctl` command:

```(bash)
journalctl -u clever
```

To run Clover software directly in the current console session, you can use the `roslaunch` command:

```(bash)
sudo systemctl restart clever
roslaunch clever clever.launch
```

You can disable Clover software autolaunch using the `disable` command:

```(bash)
sudo systemctl disable clever
```

roslaunch
---

Main documentation: http://wiki.ros.org/roslaunch.

The list of nodes / programs declared for running is specified in file `/home/pi/catkin_ws/src/clever/clever/launch/clever.launch`.

You can add your own node to the list of automatically launched ones. To do this, place your executable file (e.g. `my_program.py`) into folder `/home/pi/catkin_ws/src/clever/clever/src`. Then add the start of your node to `clever.launch`, for example:

```xml
<node name="my_program" pkg="clever" type="my_program.py" output="screen"/>
```

The started file must have *permission* to run:

```(bash)
chmod +x my_program.py
```

When scripting languages are used, [shebang] should be placed at the beginning of the file (https://ru.wikipedia.org/wiki/Shebang_(Unix)), for example:

```(bash)
#!/usr/bin/env python
```
