# Использование Optical Flow

> **Warning** Данная функция является **экспериментальной** и включена в образ с версии v0.11.4.

При использовании технологии Optical Flow возможен полет в режиме POSCTL и автономные полеты по камере, направленной вниз, за счет измерения сдвигов текстуры поверхности пола.

## Включение

> **Note** Для использования Optical Flow необходима <a id="download-firmware" href="https://github.com/CopterExpress/Firmware/releases">кастомная прошивка PX4</a>. Подробнее про прошивку см. [соответствующую статью](firmware.md).

<script type="text/javascript">
    fetch('https://api.github.com/repos/CopterExpress/Firmware/releases').then(res => res.json()).then(function(data) {
        for (let release of data) {
            if (!release.prerelease && !release.draft && release.tag_name.includes('-clever.')) {
                document.querySelector('#download-firmware').href = release.html_url;
                return;
            }
        }
    });
</script>

Необходимо использование дальномера. [Подключите и настройте дальномер VL53L1X](laser.md), используя инструкцию.

Включите Optical Flow в файле `~/catkin_ws/src/clever/clever/launch/clever.launch`:

```xml
<arg name="optical_flow" default="true"/>
```

> **Info** Для правильной работы модуль камеры должен быть корректно подключен и [сконфигурирован](camera.md).

## Настройка полетного контроллера

При использовании **EKF2** (параметр `SYS_MC_EST_GROUP` = `ekf2`):

* `EKF2_AID_MASK` – включен флажок use optical flow.
* `EKF2_OF_DELAY` – 0.
* `EKF2_OF_QMIN` – 10.
* `EKF2_OF_N_MIN` – 0.05.
* `EKF2_OF_N_MAX` - 0.2.
* `SENS_FLOW_ROT` – No rotation (отсутствие поворота).
* `SENS_FLOW_MAXHGT` – 4.0 (для дальномера VL53L1X)
* `SENS_FLOW_MINHGT` – 0.01 (для дальномера VL53L1X)
* Опционально: `EKF2_HGT_MODE` – range sensor (см. [конфигурирование дальномера](laser.md)).

При использовании **LPE** (параметр `SYS_MC_EST_GROUP` = `local_position_estimator, attitude_estimator_q`):

* `LPE_FUSION` – включены флажки fuse optical flow и flow gyro compensation.
* `LPE_FLW_QMIN` – 10.
* `LPE_FLW_SCALE` – 1.0.
* `LPE_FLW_R` – 0.1.
* `LPE_FLW_RR` – 0.0.
* `SENS_FLOW_ROT` – No rotation (отсутствие поворота).
* `SENS_FLOW_MAXHGT` – 4.0 (для дальномера VL53L1X)
* `SENS_FLOW_MINHGT` – 0.01 (для дальномера VL53L1X)
* Опционально: `LPE_FUSION` – включен флажок pub agl as lpos down (см. [конфигурирование дальномера](laser.md).

## Полет в POSCTL

Настройте POSCTL как один из полетных режимов PX4. Переведите в режим POSCTL.

## Автономный полет

Автономный полет возможен с использованием модуля [simple_offboard](simple_offboard.md).

Пример взлета на высоту 1.5 м и удержание позиции:

```python
navigate(z=1.5, frame_id='body', auto_arm=True)
```

Полет вперед на 1 м:

```python
navigate(x=1.5, frame_id='body')
```

При использовании Optical Flow возможна также [навигация по ArUco-маркерам](aruco_marker.md), в том числе [используя VPE](aruco_map.md).

## Дополнительные настройки

<!-- TODO: статья по пидам -->

Если коптер нестабильно удерживает позицию по VPE, попробуйте увеличить коэффициенты *P* PID-регулятора по скорости – параметры `MPC_XY_VEL_P` и `MPC_Z_VEL_P`.

Если коптер нестабильно удерживает высоту, попробуйте увеличить коэффициент `MPC_Z_VEL_P` или лучше подобрать газ висения – `MPC_THR_HOVER`.

Если коптер сильно уплывает по рысканью, попробуйте:

* перекалибровать гироскопы;
* перекалибровать магнитометр;
* попробовать разные значения параметра `EKF2_MAG_TYPE`, который определяет, каким образом данные с магнитометра используются в EKF2;
* изменять значения параметров `EKF2_MAG_NOISE`, `EKF2_GYR_NOISE`, `EKF2_GYR_B_NOISE`.

Если коптер уплывает по высоте, попробуйте:

* повысить значение коэффициента `MPC_Z_VEL_P`;
* изменить значение параметра `MPC_THR_HOVER`;
* выставить `MPC_ALT_MODE` = 2 (Terrain following).

## Неисправности

При появлении в QGC ошибок типа `EKF INTERNAL CHECKS` попробуйте перезагрузить EKF2. Для этого наберите в MAVLink-консоли:

```nsh
ekf2 stop
ekf2 start
```
