Просмотр изображений с камер
===

Для просмотра изображений с камер можно воспользовться [rviz](/docs/rviz.md), rqt, или смотреть их через браузер, используя web_video_server.

Настройка
---

Необходимо убедиться, что в launch-файле Клевера (`~/catkin_ws/src/clever/clever/clever.launch`) включен запуск `web_video_server`:

```xml
<arg name="web_video_server" default="true"/>
```

При изменении launch-файла необходимо перезапустить пакет `clever`:

```bash
sudo systemctl restart clever
```

Просмотр
---

Для просмотра видеострима нужно подключиться к Wi-Fi Клевера (`CLEVER-xxxx`), перейти на страницу http://192.168.11.1:8080/ и выбрать топик камеры.

TODO: иллюстрации.
