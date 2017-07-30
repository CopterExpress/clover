Предварительная настройка квадрокоптера
======

1. Скачать [QGroundCongtrol](http://qgroundcontrol.com/downloads/).

2. Подключить PixHawk к компьютеру с помощью USB-кабеля.

3. Залить прошивку PX4 последней версии. Для полетов в помещении на клевере – вариант с эстимейтором LPE.

TODO

4. Выбрать в QGroundControl раму, соответствующую раме коптера, например Quadrotor X (3DR DIY Quad) для квадрокоптера

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

10. Включить land detector (если необходим режим AUTO.LAND), изменив значения параметра COM_DISARM_LAND. Подробнее: https://dev.px4.io/en/tutorials/land_detector.html

Настройка PX4 для использования marker_navigator
===

Для полетов по полю маркеров убедиться, что:

* SYS_MC_EST_GROUP = local_position_estimator
* В LPE_FUSION установлены **только** vision position, vision yaw, land detector. При желании, можно включить Baro (барометр).
* Выключен компас: ATT_W_MAG = 0
* Включена ориентация по Yaw по зрению: ATT_EXT_HDG_M = Vision
