Подключение GPS
===

При подключении GPS появляются следующие возможности:

* Удерживание коптером позиции при полете на улице
* Программирование автономных миссий в программе QGroundControl
* Полеты на глобальные точки в автономном режиме при помощи модуля [simple offboard](simple_offboard.md).

Полезные ссылки:

* https://docs.px4.io/en/assembly/quick_start_pixhawk.html
* http://ardupilot.org/copter/docs/common-pixhawk-wiring-and-quick-start.html
* http://ardupilot.org/copter/docs/common-installing-3dr-ublox-gps-compass-module.html

Подключение
---

GPS-модуль подключается к разъемам "GPS" и "I2C" (компас) полетного контроллера.

При подключении GPS, необходимо заново откалибровать магнитометры в программе QGroundControl, подключившись по [Wi-Fi](wifi.md) или USB.

Далее, необходимо включить GPS в параметре `EKF2_AID_MASK` (при использовании EKF2) или `LPE_FUSION` (при использовании LPE).
