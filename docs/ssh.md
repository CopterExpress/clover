Доступ по SSH к Raspberry Pi
===

Основная статья: https://www.raspberrypi.org/documentation/remote-access/ssh/README.md

Если SSH не включен, его можно включить положив в boot-раздел на SD-карте файл с названием ``ssh``. При загрузке SSHD включится автоматически.

Для доступа к Raspberry Pi по SSH из Linux или macOS необходимо выполнить команду:

```bash
ssh pi@<ip-адрес-raspberry>
```

Пароль: ``raspberry``.

Для доступа по SSH из Windows можно использовать [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html).
