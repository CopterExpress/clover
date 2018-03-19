Подключение PixHawk/PixRacer к Raspberry Pi
===

Для программирования [автономных полетов](simple_offboard.md), [работы с PixHawk по Wi-Fi](gcs_bridge.md), использования [веб-пульта](web_rc.md) и других функций необходимо подсоединить Raspberry Pi к PixHawk.

Убедиться в работоспособности подключения, выполнив на Raspberry Pi:

```
rostopic echo /mavros/state
```

Поле `connected` должно содержать значение `True`.


Подключение по USB
---

Соедините PixHawk/PixRacer и Raspberry Pi micro-USB to USB кабелем.

Необходимо убедиться, что в launch-файле Клевера (`~/catkin_ws/src/clever/clever/launch/clever.launch`) тип подключения установлен на USB:

```xml
<arg name="fcu_conn" default="usb"/>
```

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

> **Hint** Для корректной работы подключения Raspberry Pi и PixHawk по USB необходимо установить значение параметра `CBRK_USB_CHK` на 197848.

Подключение по UART
---

TODO схема подключения

Необходимо убедиться, что в launch-файле Клевера (`~/catkin_ws/src/clever/clever/clever.launch`) тип подключения установлен на UART:

```xml
<arg name="fcu_conn" default="uart"/>
```

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

> **Hint** Для корректной работы подключения Raspberry Pi и PixHawk по UART необходимо установить значение параметра `SYS_COMPANION` на 921600.

Подключение к SITL
---

Для того, чтобы подсоединиться к локально/удаленно запущенному [SITL](sitl.md), необходимо установить аргумент `fcu_conn` в `udp`, и `fcu_ip` в IP-адрес машины, где запущен SITL (`127.0.0.1` для локального):

```xml
<arg name="fcu_conn" default="udp"/>
<arg name="fcu_ip" default="127.0.0.1"/>
```
