Доступ по SSH к Raspberry Pi
===

На [образе для RPi](microsd_images.md) преднастроен доступ по SSH для редактирования файлов, загрузки данных и запуска программ.

Для доступа к Raspberry Pi на Клевере по SSH из Linux или macOS необходимо запустить Терминал и выполнить команду:

```bash
ssh pi@192.168.11.1
```

Пароль: ``raspberry``.

Для доступа по SSH из Windows можно использовать [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html).

Также можно получить доступ по SSH со смартфона с помощью приложения [Termius](https://www.termius.com).

Подробнее: https://www.raspberrypi.org/documentation/remote-access/ssh/README.md

Веб-доступ
----------

Начиная с версии 0.11.4 [образа](microsd_images) доступ к шелу также доступен через веб-браузер (с использованием [Butterfly](https://github.com/paradoxxxzero/butterfly)). Для доступа откройте страницу http://192.168.11.1 и выберите на ней ссылку *Open web terminal*:

<img src="assets/butterfly.png">
