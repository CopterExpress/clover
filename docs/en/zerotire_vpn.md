# Creating a virtual network ZeroTire One and connecting to it

## Creating and configuring a ZeroTire network

1. Go to [ZeroTire](https://www.zerotier.com/) website.

    <img src="../assets/zerotire/login_1.png" width=300 class="zoom border center">

2. Sign up on ZeroTire.

    <img src="../assets/zerotire/login_2.png" width=300 class="zoom border center">

3. Go to your account.

4. Click on the *Create A Network*.

    <img src="../assets/zerotire/network_1.png" width=300 class="zoom border center">

5. After that, you will see the network you created, its ID and name. Click on the network to configure it.

    <img src="../assets/zerotire/network_2.png" width=300 class="zoom border center">

6. In the window that appears you can change the network name and connection privacy.

    <img src="../assets/zerotire/network_3.png" width=300 class="zoom border center">

7. Scroll down to the *Members*column. It will say that there are no users on the network.

    <img src="../assets/zerotire/network_4.png" width=300 class="zoom border center">

8. Devices connected to the network will be displayed in this column. To allow them to connect to the network, activate the *Auth?* checkbox. The connected device will automatically be given an internal IP address, which will then be used to communicate with this device.

    <div class="image-group">
        <img src="../assets/zerotire/network_5.png" width=300 class="zoom border">
        <img src="../assets/zerotire/network_6.png" width=300 class="zoom border">
    </div>

    > **Hint** specify names for new devices, it will help you distinguish them from each other in the future.

9. Repeat the last step for all the devices that you want to connect.

    > **Info** ZeroTire network supports up to 50 users simultaneously for free use.

## Setup on Windows

### Installing the app

1. Go to the ZeroTire website.

    <img src="../assets/zerotire/download_1.png" width=300 class="zoom border center">

2. Click on the Windows icon.

    <img src="../assets/zerotire/download_2.png" width=300 class="zoom border center">

3. Download and run the `Zero Tare One.msi` file.

    <div class="image-group">
        <img src="../assets/zerotire/windows_1.png" width=300 class="zoom border">
        <img src="../assets/zerotire/windows_2.png" width=300 class="zoom border">
    </div>

### Network connection

1. Run ZeroTire One.

2. Click on the ZeroTire One icon in the taskbar.

3. Click on the *Join Network...* to connect to the network.

    <img src="../assets/zerotire/windows_3.png" width=300 class="zoom border center">

4. In the window that appears, enter your network ID and click *Join*.

    <img src="../assets/zerotire/windows_4.png" width=300 class="zoom border center">

5. Allow using the new network.

    <img src="../assets/zerotire/windows_5.png" width=300 class="zoom border center">

## Setup on iOS

### Installing the app

1. Go to the ZeroTire website.

    <img src="../assets/zerotire/download_1.png" width=300 class="zoom border center">

2. Click on the iOS icon.

    <img src="../assets/zerotire/download_2.png" width=300 class="zoom border center">

3. Install the *ZeroTire One* app.

### Network connection

1. Run ZeroTire One app.

2. Click on *+* to add a new connection.

    <img src="../assets/zerotire/ios_1.png" width=300 class="zoom border center">

3. Confirm the privacy policy.

    <img src="../assets/zerotire/ios_2.png" width=300 class="zoom border center">

4. Enter your network ID and click *Add Network*.

    <img src="../assets/zerotire/ios_3.png" width=300 class="zoom border center">

5. Confirm adding the new VPN configuration.

6. Connect to the VPN network by sliding the network activation slider.

    <div class="image-group">
        <img src="../assets/zerotire/ios_4.png" width=300 class="zoom border">
        <img src="../assets/zerotire/ios_5.png" width=300 class="zoom border">
    </div>

## Setup on Linux

### Installing the app

1. Open the console by pressing the keyboard shortcut *ctrl + alt + t* or type *terminal* in the program search bar.

2. Enter the Zero Tare installation command.

    ```bash
    curl -s  https://install.zerotier.com  | sudo bash
    ```

### Network connection

1. Open the console.

2. Enter the command `sudo zerotire-cli join network-id`, where `network-id` is your network ID.

    <img src="../assets/zerotire/linux_1.png" width=300 class="zoom border center">

3. If the connection is successful, the corresponding message will be displayed in the console.

## Installing and configuring on macOS

### Installing the app

1. Go to the ZeroTire website.

    <img src="../assets/zerotire/download_1.png" width=300 class="zoom border center">

2. Click on the macOS icon.

    <img src="../assets/zerotire/download_2.png" width=300 class="zoom border center">

3. Download and run `ZeroTire One.pkg` file.

4. Install the ZeroTire One app.

### Network connection

1. Run ZeroTire One app.

2. Click on the ZeroTire One icon in the taskbar .

3. In the window that appears, click on *Join Network...*.

    <img src="../assets/zerotire/macos_1.png" width=300 class="zoom border center">

4. In the *Enter Network ID* field, enter your network ID.

    <img src="../assets/zerotire/macos_2.png" width=300 class="zoom border center">
