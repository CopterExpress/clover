Copter Hack 2017
===

Модули
---

* Навигация в поле маркеров и упрощенное управление коптером: https://github.com/CopterExpress/marker_navigator (установлено на флешке)

* ROS-модуль для взаимодействия с сервером воспроизведения музыки https://github.com/CopterExpress/copter_hack_music (установлено на флешке)

* Исходник сервера воспроизведения музыки https://github.com/CopterExpress/copter_hack_music_server

Просмотр видео с камеры
---

Выполнить на Raspberry:

```bash
rosrun web_video_server web_video_server
```

Открыть в браузере странцу http://<ip raspberry>:8080

Полезные статьи
---

* [Настройка коптера](setup.md)

* [Полетные режимы](modes.md)

* [Пакет MAVRos](mavros.md)

* Неплохая вводная статья
https://habrahabr.ru/post/227425/ 

* Сигналы, применяющиеся в дронах
https://geektimes.ru/post/258186/ 

* Хорошая статья про ПИДы
https://habrahabr.ru/company/technoworks/blog/216437/

* Запись видеолекций
https://copterexpress.timepad.ru/event/510375/

* [Aubio](https://aubio.org), библиотека для анализа звука (музыки)

* Пакеты для работы с музыкой для Python https://wiki.python.org/moin/PythonInMusic
