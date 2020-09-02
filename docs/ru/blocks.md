# Блочное программирование

TODO

Набор блоков приблизительно аналогичен набору ROS-сервисов [API автономных полетов Клевера](simple_offboard.md).

<img src="../assets/blocks/blocks.png" width=600>

## Блоки

В этом разделе приведено описание некоторых блоков.

### take_off

Взлететь на указанную высоту в метрах. Высота может быть произвольным блоком, возвращающим числовое значение.

Флаг `wait` определяет, должен ли дрон ожидать окончания взлета перед выполнением следующего блока.

<img src="../assets/blocks/take-off.png" srcset="../assets/blocks/take-off.png 2x">

### navigate

Прилететь в заданную точку.

Флаг `wait` определяет, должен ли дрон ожидать завершения полета в точку перед выполнением следующего блока.

<img src="../assets/blocks/navigate.png" srcset="../assets/blocks/navigate.png 2x">

### land

Произвести посадку дрона.

Флаг `wait` определяет, должен ли дрон ожидать окончания посадки перед выполнением следующего блока.

<img src="../assets/blocks/land.png" srcset="../assets/blocks/land.png 2x">

### wait

Ожидать заданное время в секундах. Время ожидания может быть произвольным блоком, возвращающим числовое значение.

<img src="../assets/blocks/wait.png" srcset="../assets/blocks/wait.png 2x">

### get_position

<img src="../assets/blocks/get-position.png" srcset="../assets/blocks/get-position.png 2x">

### set_effect

<img src="../assets/blocks/set-effect.png" srcset="../assets/blocks/set-effect.png 2x">

<img src="../assets/blocks/random-color.png" srcset="../assets/blocks/random-color.png 2x">
