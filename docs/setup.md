Предварительная настройка квадрокоптера
======

1. Скачать [QGroundCongtrol](http://qgroundcontrol.com/downloads/).

2. Подключить PixHawk к компьютеру с помощью USB-кабеля.

3. Залить прошивку PX4 последней версии. Для полетов в помещении на клевере – вариант с эстимейтором LPE.

TODO

4. Выбрать раму, соответствующую раме коптера, например Quadrotor X (3DR DIY Quad) для квадрокоптера

TODO

5. Провести калибровки IMU, компаса, радиопульта.

TODO

Настройка для автономных полетов с использованием companion-computer (Raspberry Pi 3)
===

Основная статья: https://dev.px4.io/en/ros/offboard_control.html

6. Убедиться в коррекном подключении Raspberry Pi и PixHawk по UART.

7. Установить значение SYS_COMPANION на 921600 в параметрах.

8. Убедиться в работоспособности подключения, выполнив с Raspberry Pi:

```bash
rostopic echo /mavros/state
```

9. Отключить Safety Switch, если он не установлен, поменяв параметр CBRK_IO_SAFETY на 22027.

