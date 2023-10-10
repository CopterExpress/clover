# Система радио-навигации

[CopterHack-2023](copterhack2023.md), команда **C305**.

## Информация о команде

Мы команда студентов, представляющая Центр Проектной Деятельности Дальневосточного федерального университета (C305).

Состав команды:

* Антонов Георгий, @SonSobaki, инженер.
* Филимонов Сергей, @Lukerrr, программист.
* Смадыч Никита, @Sm_nikita, инженер-программист.
* Максим Харченко, @milian_c305, программист.

## Описание проекта

### Идея проекта

Проект направлен на разработку системы позиционирования внутри помещений для квадрокоптеров с использованием широкополосных передатчиков DWM1000.

Разрабатываемая система позиционирования использует передатчики DWM1000, которые обеспечивают широкополосную связь на радиочастотах. Она предназначена для обеспечения точного и надежного позиционирования квадрокоптеров внутри зданий, где оптические системы могут оказаться ограниченными или неэффективными.

Одной из ключевых особенностей этой системы является ее способность обеспечивать высокую точность позиционирования и дальность действия. Широкополосные передатчики позволяют достичь высокой разрешающей способности и низкой задержки передачи данных, что особенно важно в случае быстрого и точного позиционирования квадрокоптеров. Благодаря использованию радиочастотных сигналов, система не подвержена помехам от окружающей среды, таких как освещение или преграды. Это обеспечивает стабильную работу системы в различных условиях и позволяет использовать ее в помещениях с ограниченной видимостью или сложным рельефом.

В целом, разрабатываемая система предлагает более точное, надежное и экономичное решение по сравнению с оптическими системами.

### Ключевые особенности

* Точность системы позиционирования +-0.1м.
* Поддержка indoor и outdoor навигации.
* Лёгкая масштабируемость системы.
* Лёгкое развёртывание и лёгкая настройка системы.

### Презентационный ролик

[![](https://img.youtube.com/vi/ra45vH3IFuI/0.jpg)](https://www.youtube.com/watch?v=ra45vH3IFuI)

### Документация к проекту

* [Работа с UWB модулями](https://github.com/NikitaS2001/dw1000-stm32/blob/main/README.md)
* [Запуск ROS пакета позиционирования](https://github.com/NikitaS2001/dwm1000_pose/blob/main/README.md)
* [Настройка системы позиционирования](https://github.com/NikitaS2001/dwm1000_pose/blob/main/docs/ru/navigation_system_setup.md)

### Ресурсы проекта

* [Исходный код прошивок UWB модулей](https://github.com/NikitaS2001/dw1000-stm32)
* [Исходный код ROS пакета позиционирования](https://github.com/NikitaS2001/dwm1000_pose)
* [Модели корпуса UWB модулей](https://github.com/NikitaS2001/dw1000-stm32/tree/main/3D)