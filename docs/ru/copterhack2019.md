# Copter Hack 2019

Хакатон [Copter Hack 2019](https://copterexpress.timepad.ru/event/768108/) проходит 11–13 октября в Технополисе "Москва".

Официальный сайт: https://ru.coex.tech/copterhack.

Чат хакатона: https://t.me/CopterHack.

Timepad: https://copterexpress.timepad.ru/event/1017592/.

## Информация для участников

### Особенности настройки полетного контроллера COEX PIX

При использовании полетного контроллера *COEX Pix* перед калибровкой датчиков вам стоит обратить внимание, что в графе *Autopilot orientation* вы должны выбрать параметр `ROTATION_ROLL_180_YAW_90`. Данную настройку требуется проводить при калибровке каждого из датчиков.

<img src="../assets/autopilot_orientation.png" class="center" width=600>

Этот параметр устанавливается для того, чтобы на программном уровне настроить ориентацию вашего *IMU* датчика находящегося на полетном контроллере.

### Полет с использованием Optical Flow

При полете только с использованием Optical Flow необходимо в QGroundControl в параметре `LPE_FUSION` включить галочку `pub agl as lpos down`.

Необходимо также убедиться, что лазерный дальномер корректно установлен и работает (см. [конфигурирование дальномера](laser.md)).

## Лекции

Лекция 1: введение – https://www.youtube.com/watch?v=cjtmZNuq7z0.

Лекция 2: настройка полетного контроллера – https://www.youtube.com/watch?v=PJNDYFPZQms.

Лекция 3: архитектура полетного контроллера PX4 – https://www.youtube.com/watch?v=_jl7FImq3jk.

Лекция 4: автономные полеты: https://www.youtube.com/watch?v=ThXiNG1IzvI.

См. также другие видео на канале COEX на YouTube: https://www.youtube.com/channel/UCeCu93sLBkcgbIkIC7Jaauw/featured.
