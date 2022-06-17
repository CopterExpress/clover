# Сборка и настройка электромагнитного захвата

Магнитный захват можно собрать различными способами в соответствии с электрической схемой.

<img src="../assets/magnet_grip/scheme.jpg" width=300 class="zoom border center">

Ниже представлен пример сборки схемы электромагнитного захвата на макетной плате.

> **Info** Рекомендуется проложить проводку между элементами с обратной стороны платы (на дальнейших изображениях проводка сделана поверх схемы, для наглядности).

1. На паечной плате разместите диод Шоттки, резистор на 10 кОм и транзистор.

    <img src="../assets/magnet_grip/magnet1.png" width=300 class="zoom border center">

2. Припаяйте контакты с другой стороны платы и откусите оставшиеся ножки элементов.
3. Соедините контакты резистора и двух крайних ножек транзистора.

    <img src="../assets/magnet_grip/magnet2.png" width=300 class="zoom border center">

4. Соедините центральную ножку транзистора и ножку диода Шоттки (противоположную серой маркировочной полоске).

    <img src="../assets/magnet_grip/magnet3.png" width=300 class="zoom border center">

5. Обрежьте необходимое количество провода магнитного захвата и припаяйте его к контактам диода Шоттки.

    <img src="../assets/magnet_grip/magnet4.png" width=300 class="zoom border center">

6. Припаяйте провода *Dupont*-папа к ножке транзистора и диода (красный, черный провода), а также провод *Dupont*-мама на противоположную ножку транзистора (белый провод).

    <img src="../assets/magnet_grip/magnet5.png" width=300 class="zoom border center">

## Проверка работы электромагнитного захвата

Для того, чтобы проверить работу захвата, подайте на сигнальный провод напряжение 5В. Для этого можно использовать провод *Dupont* папа-папа.

<img src="../assets/magnet_grip/magnet_check.png" width=300 class="zoom border center">

После подачи напряжения магнит должен включиться.

## Подключение к Raspberry Pi

Подключите магнитный захват к плате Raspberry Pi для программного использования

<img src="../assets/magnet_grip/magnet_raspberry.png" width=300 class="zoom border center">

Пример кода, активирующего магнитный захват, можно посмотреть [тут](gpio.md#подключение-электромагнита).

## Подключение к Arduino

Подключите захват плате Arduino Nano, чтобы использовать его в ручном режиме.

Удобно ее располагать на той же паечной плате -- вставьте ее в подходящие отверстия и припаяйте с обратной стороны к плате.

<img src="../assets/magnet_grip/magnet_arduino1.png" width=300 class="zoom border center">

Затем подключите сигнальный выход схемы к выбранному порту и припаяйте провод *Dupont*-мама к выбранному сигнальному порту на плате.

<img src="../assets/magnet_grip/magnet_arduino2.png" width=300 class="zoom border center">

## Установка электромагнитного захвата

1. В центральное отверстие на деке захвата установите электромагнит.
2. Стяжкой притяните собранную схему к обратной стороне деки.
3. Сигнальный вывод Arduino *D11* вставьте в один из выводов *AUX* на полетном контроллере.
4. Вставьте силовой вывод электромагнитного захвата в JST 5В.

## Настройка электромагнитного захвата

Для управления магнитом через плату Arduino Nano, используйте код ниже:

```cpp
void setup() {
  pinMode(11, INPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  if (int duration = pulseIn(11, HIGH) > 1200) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}
```

Для однозначного определения статуса магнитного захвата, можно подключить светодиодную ленту типа *ws281x* (входит в наборы "Клевер"). Подключите ее к питанию +5v – 5v, земле GND – GND и сигнальный контакт DIN – Arduino D12.

Для управления магнитным захватом со светодиодной лентой через плату Arduino Nano используйте код ниже:

```cpp
#include <Adafruit_NeoPixel.h>
#define NUMPIXELS 72
#define PIN 12
int pin = 11;
int led = 13;

unsigned long duration;
Adafruit_NeoPixel strip (NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(10);
  Serial.begin(9600);
  pinMode(pin, INPUT);
  pinMode(led, OUTPUT);
}

void loop() {
  duration = pulseIn(pin, HIGH);
  Serial.println(duration);
  delay(100);
  if (duration >= 1500) {
    digitalWrite(led, HIGH);
    for (int i = -1; i < NUMPIXELS; i++) {
      strip.setPixelColor(i, strip.Color(255, 0, 0));
      strip.show();
    }
  } else { 
    digitalWrite(led, LOW);
    for (int i = -1; i < NUMPIXELS; i++) {
      strip.setPixelColor(i, strip.Color(0, 255, 0));
      strip.show();
    }
  }
}
```
