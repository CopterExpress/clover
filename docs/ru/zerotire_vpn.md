# Создание и подключение к виртуальной сети ZeroTire

## Создание и управление сетью ZeroTire

1. Зайдите на сайт [ZeroTire](https://www.zerotier.com/).

    <img src="../assets/zerotire/login_1.png" width=300 class="zoom border center">

2. Зарегистрируйтесь в ZeroTire.

    <img src="../assets/zerotire/login_2.png" width=300 class="zoom border center">

3. Зайдите в свой аккаунт.

4. Нажмите кнопку *Create A Network*.

    <img src="../assets/zerotire/network_1.png" width=300 class="zoom border center">

5. После этого вы увидите созданную вами сеть, ее ID и название. Для настройки сети нажмите на нее.

    <img src="../assets/zerotire/network_2.png" width=300 class="zoom border center">

6. В открывшемся окне можно изменить имя сети и приватность подключения.

    <img src="../assets/zerotire/network_3.png" width=300 class="zoom border center">

7. Пролистайте ниже, до графы *Members*. В ней будет написано о том, что в сети нету пользователей.

    <img src="../assets/zerotire/network_4.png" width=300 class="zoom border center">

8. Устройства подключенные к сети будут отображаться в данной графе, для того, чтобы позволить им подключиться к сети, активируйте чекбокс *Auth?*. При этом, подключенному устройству автоматически выдастся внутренний IP адрес, в дальнейшем он будет использоваться для связи с данным устройством.

    <div class="image-group">
        <img src="../assets/zerotire/network_5.png" width=300 class="zoom border">
        <img src="../assets/zerotire/network_6.png" width=300 class="zoom border">
    </div>

    > **Hint** Указывайте имена для новых устройств, этом поможет вам в дальнейшем отличать их друг от друга.

9. Повторите последний шаг для всех подключаемых устройств.

    > **Info** Сеть ZeroTire в случае бесплатного использования поддерживает до 50 пользователей одновременно.

## Настройка на Windows

### Установка

1. Перейдите на сайт ZeroTire.

    <img src="../assets/zerotire/download_1.png" width=300 class="zoom border center">

2. Нажмите на иконку Windows.

    <img src="../assets/zerotire/download_2.png" width=300 class="zoom border center">

3. Скачайте и запустите файл `ZeroTire One.msi`.

    <div class="image-group">
        <img src="../assets/zerotire/windows_1.png" width=300 class="zoom border">
        <img src="../assets/zerotire/windows_2.png" width=300 class="zoom border">
    </div>

### Настройка

1. Запустите ZeroTire One.

2. Нажмите на иконку ZeroTire One в панели задач.

3. Нажмите на кнопку *Join Network...* для подключения к сети.

    <img src="../assets/zerotire/windows_3.png" width=300 class="zoom border center">

4. В появившемся окне введите ID вашей сети и нажмите кнопку *Join*.

    <img src="../assets/zerotire/windows_4.png" width=300 class="zoom border center">

5. Разрешите использование новой сети.

    <img src="../assets/zerotire/windows_5.png" width=300 class="zoom border center">

## Настройка на iOS

### Установка

1. Перейдите на сайт ZeroTire.

    <img src="../assets/zerotire/download_1.png" width=300 class="zoom border center">

2. Нажмите на иконку iOS.

    <img src="../assets/zerotire/download_2.png" width=300 class="zoom border center">

3. Установите приложение *ZeroTire One*.

### Настройка

1. Запустите приложение *ZeroTire One*.

2. Нажмите на *+* для добавления нового подключения.

    <img src="../assets/zerotire/ios_1.png" width=300 class="zoom border center">

3. Подтвердите политику конфиденциальности.

    <img src="../assets/zerotire/ios_2.png" width=300 class="zoom border center">

4. Введите ID вашей сети и нажмите кнопку *Add Network*.

    <img src="../assets/zerotire/ios_3.png" width=300 class="zoom border center">

5. Подтвердите добавление новой конфигурации VPN.

6. Подключитесь к VPN сети, сдвинув ползунок активации сети.

    <div class="image-group">
        <img src="../assets/zerotire/ios_4.png" width=300 class="zoom border">
        <img src="../assets/zerotire/ios_5.png" width=300 class="zoom border">
    </div>

## Настройка на Linux

### Установка

1. Откройте консоль, для этого нажмите сочетание клавиш *Ctrl* + *Alt* + *T* или в строке поиска программ введите *Terminal*

2. Введите команду установки ZeroTire.

    ```bash
    curl -s https://install.zerotier.com | sudo bash
    ```

### Настройка

1. Откройте консоль.

2. Введите команду `sudo zerotire-cli join network-id`, где `netwirk-id` это ID вашей сети.

    <img src="../assets/zerotire/linux_1.png" width=300 class="zoom border center">

3. При успешном подключении, в консоль будет выведено соответствующее сообщение.

## Установка и настройка на macOS

### Установка

1. Перейдите на сайт ZeroTire.

    <img src="../assets/zerotire/download_1.png" width=300 class="zoom border center">

2. Нажмите на иконку macOS.

    <img src="../assets/zerotire/download_2.png" width=300 class="zoom border center">

3. Скачайте и запустите файл `ZeroTire One.pkg`.

4. Установите приложение ZeroTire One.

### Настройка

1. Запустите приложение ZeroTire One.

2. В панеле задач нажмите на иконку ZeroTire One.

3. В открывшемся окне нажмите *Join Network...*.

    <img src="../assets/zerotire/macos_1.png" width=300 class="zoom border center">

4. В поле *Enter Network ID* введите ID вашей сети.

    <img src="../assets/zerotire/macos_2.png" width=300 class="zoom border center">
