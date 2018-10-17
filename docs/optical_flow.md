# Использование Optical Flow

> **Warning** Данная функция является **экспериментальной** и включена в образ с версии v0.11.4.

При использовании технологии Optical Flow возможен полет в режиме POSCTL и автономные полеты по камере, направленной вниз, засчет измерения сдвигов текстуры поверхности пола.

## Включение

А данный момент для использования Optical Flow необходима [кастомная прошивка PX4](https://yadi.sk/d/KaxaIhohu4V8XA).

Необходимо использования дальномера. При использовании дальномера STM vl53l1x, необходимо подключить его к Raspberry Pi по I2C и включить его в `~/catkin_ws/src/clever/clever/launch/clever.launch`:

```xml
<arg name="vl53l1x" default="true"/>
```

Необходимо включить Optical Flow:

```xml
<arg name="optical_flow" default="true"/>
```

В `main_camera.launch` должен быть выставлен корректный фрейм камеры.

Рекомендуемые параметры PX4:

* `SYS_MC_EST_GROUP` – 2 (EKF2).
* `EKF2_AID_MASK` – use optical flow.
* `EKF2_OF_DELAY` – 0.
* `EKF2_OF_QMIN` – 20.
* `EKF2_HGT_MODE` – range sensor (ремменд.).

## Полет в POSCTL

Настройте POSCTL как один из полетных режимов PX4. Переведите в режим POSCTL.

## Автономный полет

Автономный полет возможен с использованием модуля [simple_offboard](simple_offboard.md).

## Troubleshooting

При появлении в QGC ошибок типа `EKF INTERNAL CHECKS` попробуйте перезагрузить ekf2. Для этого наберите в MAVLink-консоли:

```nsh
ekf2 stop
ekf2 start
```
