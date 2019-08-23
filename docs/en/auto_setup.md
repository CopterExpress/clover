# Step-by-step guide on autonomous flight with Clever 4

The guid refers to many other articles

This manual contains links to other articles in which each of the topics addressed is discussed in more detail. If you encounter difficulties while reading one of these articles, it is recommended that you return to this manual, since many operations here are described step by step and some unnecessary steps are skipped.

## Raspberry Pi initial setup

- Install Raspberry Pi and a camera on the drone according to the [manual](assemble_3.md # montage-raspberry).
- Download the system image [here](image.md).
- Burn the image to the microSD card.
- Insert the card into Raspberry Pi.
- Connect power to Raspberry Pi and wait for the Wi-Fi network to appear. To do this, connect the Raspberry Pi to the computer via the microUSB cable.
   On Raspberry Pi, the green LED should flash blink. It shows that Raspberry Pi works properly.

  > **Warning** Before connecting the Raspberry Pi to the computer via USB, you need to remove the 5V power cable from Raspberry Pi. Otherwise, there may be problems with power.

- Connect to Wi-Fi and open the web interface ([this article] (wifi.md)).

   After the first power-up, the network appears with a delay. You need to wait until the system is fully loaded. If the Clever network does not appear in the list of networks for a long time, reopen the window with the network selection. Then the list of networks will be updated.

> **Hint** Now if you have connected to the Clever's Wi-Fi network, it is recommended to open the [local version of this guide](http://192.168.11.1/docs/ru/auto_setup.html), otherwise the links will not work.

- Connect to Raspberry Pi via SSH.

   Web access is the easiest way. Follow the instructions in the article [SSH Access] (ssh.md # web access).

- You can change the name and password of the network if you want to. See the article "[Network Settings] (network.md # change-password-or-ssid-network-name)". The remaining operations with the network are unnecessary.

- Use the nano editor to edit files. [Instructions for working with nano] (editing.md).

  > **Hint** In nano, you can only move the cursor with the arrow keys on the keyboard.

- Reboot Raspberry Pi:

  ```bash
  sudo reboot
  ```

  The connection will temporary close, a new network will be created and you will need to reconnect to it.

- Make sure that the camera is working correctly. Follow the link http://192.168.11.1:8080 and click `image_raw`.

  For more information read "[Viewing images from cameras](web_video_server.md)".

  If the image is blurry, you need to focus the lens. To do this, twist the lens in one or the other direction. Continue to twist until the image becomes clear.

  > **Hint** The red LED on the camera should be lit: it means that the camera is currently capturing image. If the LED does not light: either the camera is connected incorrectly, or the operating system did not boot yet, or there is an error in settings.

## Basic commands

You will need the basic Linux commands, as well as special Clever commands, to work efficiently in the system.

Show list of files and folders:

```bash
ls
```

Go to certain directory by entering the path too it (catkin_ws/src/clever/clever/launch/):

```bash
cd catkin_ws/src/clever/clever/launch/
```

Go to home directory:

```bash
cd
```

Open the file `file.py`:

```bash
nano file.py
```

Open the file clever.launch by entering the full path to it (it works even if you're in a different directory):

```bash
nano ~/catkin_ws/src/clever/clever/launch/clever.launch
```

Save file (press sequentially):

```bash
Ctrl+X; Y; Enter
```

Delete a file or folder called `name` (WARNING: the operation will not request confirmation. Be careful!):

```bash
rm -rf name
```

Make a new directory called `myfolder`:

```bash
mkdir myfolder
```

Raspberry Pi complete reboot:

```bash
sudo reboot
```

Reboot only Clever package:

```bash
sudo systemctl restart clever
```

Perform selfcheck:

```bash
rosrun clever selfcheck.py
```

Stop a program:

```bash
Ctrl+C
```

Start a program `myprogram.py` using Python:

```bash
python myprogram.py
```

Journal of the events related to Clever package. Scroll the list by pressing Enter or Ctrl+V (scrolls faster):

```bash
journalctl -u clever
```

Open the sudoers file with super user rights (this particular file doesn't open without sudo. You can use sudo to open other locked files or run programs that require super user rights):

```bash
sudo nano /etc/sudoers
```

## Setting Raspberry Pi for autonomous flight

Most of the parameters for autonomous flight are located in the following directory: `~/catkin_ws/src/clever/clever/launch/`.

- Enter the directory:

  ```bash
  cd ~/catkin_ws/src/clever/clever/launch/
  ```

  The `~` symbol stands for home directory of your user. If you are already in the directory, you can go with just the command:
`cd catkin_ws/src/clever/clever/launch/`

  > **Hint** Tab can automatically complete the names of files, folders or commands. You need to start entering the desired name and press Tab. If there are no conflicts, the name will be auto completed. For example, to quickly enter the path to the `catkin_ws/src/clever/clever/launch/` directory, after entering `cd`, you can start typing the following key combination:`c-Tab-s-Tab-c-Tab-c-Tab-l-Tab`. This way you can save a lot of time when writing a long command, and also avoid possible mistakes in writing the path.

- In this folder you need to configure three files:

  - `clever.launch`
  - `aruco.launch`
  - `main_camera.launch`

- Open the file `clever.launch`:

  ```bash
  nano clever.launch`
  ```

  You must be in the directory in which the file is located. If you are in other directory, you can open the file by writing the full path to it:

  ```bash
  nano ~/catkin_ws/src/clever/clever/launch/clever.launch
  ```

  If two users are editing a file at the same time, or if previously the file was closed incorrectly, nano will not display the file contents, it will ask for permission to display the file. To grant permission, press Y.

  If the content of a file is still empty, you may have entered the file name incorrectly. You need to pay attention to the extension. If you entered a wrong name or extension, nano will create a new empty file named this way, which is undesirable. Such file should be deleted.

- Find the following line in clever.launch file:

  ```xml
  <arg name="aruco" default="false"/>
  ```

  Replace `false` with `true`:

  ```xml
  <arg name="aruco" default="true"/>.
  ```

  This will acticate the ArUco marker detection module. 
- Open the file `aruco.launch`:

  ```bash
   nano aruco.launch
  ```

- Here you need to activate some parameters. Go to the [article](aruco_map.md) for more detail.

  Here is what you should get:

  ```
  <arg name="aruco_detect" default="true"/>
  <arg name="aruco_map" default="true"/>
  <arg name="aruco_vpe" default="true"/>`
  ```

- Generate the ArUco markers field. See the article [Map-based navigation with ArUco markers] (aruco_map.md # marker map settings) for details. To generate markers, you need to enter a command with specific values.

  Here is the example generating command where: 

  - marker length = 0.335 m (`length`)
  - 10 columns (x)
  - 10 rows (y)
  - distance between markers on the x axis = 1 m (`dist_x`)
  - distance between markers on the y axis = 1 m (`dist_y`)
  - the first marker's ID = 0 (`first`)
  - the marker map name is default: map.txt
  - the marker map numbering is from the top left corner (key `--top-left`)

  ```bash
  rosrun aruco_pose genmap.py 0.335 10 10 1 1 0 > ~/catkin_ws/src/clever/aruco_pose/map/map.txt --top-left
  ```

  In most maps, numbering starts with a zero marker. Also, in most cases, numbering starts from the upper left corner, so when generating, it is very important to enter the key `--top-left`.

  > **Hint** If you choose a different name for your ArUco map, you also need to  change it in the `aruco.launch`. Find the line
`<param name="map" value="$(find aruco_pose)/map/map.txt"/>`
and replace map.txt with your map name.

- Edit the `main_camera.launch` for setting the camera:

  Read more in the article. "[Camera orientation](camera_frame.md)".

  In this file, you need to edit the line with the camera location parameters. The line looks like this:

  ```xml
  <node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 -1.5707963 0 3.1415926 base_link main_camera_optical"/>
  ```

  In the file you will find many lines similar to this, but most of them are commented out (i.e. not readable) and only one is uncommented. These are pre-configured settings from which you can choose the one you need.

  Comment in XML is `<!--` at the beginning of a line and   `-->` at the end of a line. An example of a commented line:

  ```xml
  <!--<node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 -1.5707963 0 3.1415926 base_link main_camera_optical"/>-->
  ```

  An example of an uncommented line (the line will be read by the program):

  ```xml
  <node pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 -1.5707963 0 3.1415926 base_link main_camera_optical"/>
  ```

  The writing above each of these lines indicates which camera position the line corresponds to. If the camera cable goes forward relative to the drone, and the camera is pointing down, you need to select the following setting:

  ```xml
  <!-- camera is oriented downward, camera cable goes forward  [option 2] -->
  ```

  To select the desired setting, you need to uncomment the corresponding line, and comment out another similar line so that there are no conflicts.

- Save changes. Press sequentially:

  ```
  Ctrl+x; y; Enter
  ```

- Перезагрузите модуль Клевер:

  ```bash
  sudo systemctl restart clever
  ```

## Setting the flight controller

- Flash the flight controller with modified firmware. You can download it [here](setup.md) in the section "Flashing the flight controller".

- Instructions for flashing and calibrating the flight controller are in the same article.

> **Warning** Don't forget to choose the downloaded firmware when you flash the flight controller.

## Connecting the flight controller with Raspberry Pi

- Connect the Raspberry Pi and the Pixracer via the microUSB cable. The cable should be tightly fastened and passed through the bottom of the drone to not get into the propellers.

- Connect remotely to the flight controller through QGroundControl.
   All the necessary settings are already set in the Clever settings. Now you need to create a new connection in QGroundControl

остается лишь создать новое подключение в QGroundControl, выбрать его и подключиться. Настраивается оно, как на картинке в статье "[Подключение QGroundControl по Wi-Fi](gcs_bridge.md)".

## Настройка пульта

- Настройка полетных режимов описана в статье "[Полетные режимы](modes.md)".

  Канал 5 должен располагаться на переключателе SwC; Канал 6 - на SwA. Однако вы можете настроить эти каналы любым удобным для вас образом.

## Выполнение автоматической проверки

Проверку следует выполнить, когда вы полностью настроили дрон, а также при возникновении неполадок. Подробно процедура описана в статье "[Автоматическая проверка](selfcheck.md)".

- Выполнить команду:

  ```bash
  rosrun clever selfcheck.py
  ```

## Написание программы

В статье "[Автономный полет](simple_offboard.md)" описана работа с модулем `simple_offboard`, который создан для простого программирования дрона. В ней даны описания основных функций, а также примеры кода.

- Скопируйте из раздела "Использование из языка Python" пример кода и вставьте в редактор (например, в Visual Studio Code, PyCharm, Sublime Text, Notepad++).

- Сохраните документ с расширением .py для включения подсветки текста.

- Далее необходимо добавить полётные команды в программу. Примеры таких команд представлены в статье. Нужно написать функции для взлета и полета в точку, а также для посадки.

- Взлет.

  Для взлета можно использовать функцию `navigate`:

  ```python
  navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
  ```

  Добавьте эту строку внизу программы.

  Также добавьте команду ожидания:

  ```python
  rospy.sleep(3)
  ```

> **Hint** Важно выделить время на выполнение команды `navigate`, иначе коптер, не дожидаясь выполнения предыдущей команды, сразу перейдет к выполнению следующей. Для этого используется команда `rospy.sleep()`. В скобках указывается время в секундах. Функция `rospy.sleep()` относится к предыдущей команде `navigate`, а не к последующей, то есть это время, которое мы даем на то, чтобы долететь до точки, обозначенной в предыдущем `navigate`.

- Зафиксировать положение дрона в системе координат маркерного поля.

  Для этого нужно выполнить `navigate` и указать в нем необходимые координаты (например, x=1, y=1, z=1.5) и выбрать систему координат (`frame_id`):

  ```python
  navigate(x=1, y=1, z=1.5, speed=1, frame_id='aruco_map')
  ```


- As the result you will get:

  ```python
  navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
  rospy.sleep(3)
  navigate(x=1, y=1, z=1.5, speed=1, frame_id='aruco_map')
  ```

  > **Warning** Note that the parameter `auto_arm = True` is only  set once on the first take-off. In other cases it should not be set True because it will prevent overtaking the control.

- If you want to add other points for the drone's flight, you need to add another `navigate` and`rospy.sleep ()`. Time must be calculated separately for each point, depending on the speed of flight and the distance between the points.

  For example, if we want to reach the point (3, 3, 1.5):

  ```python
  navigate(x=3, y=3, z=1.5, speed=1, frame_id=‘aruco_map’)
  rospy.sleep(3)
  ```

  > **Warning** Coordinates should not exceed the size of your field. If the field is 4x4 meters in size, the maximum coordinate value should be 4.

- After reaching all of the points you need to land. The following line is placed at the end of the program:

  ```
  land()
  ```

## Sending the program on the drone

The easiest way to send the program is to copy the content of the program, create a new file on the Clever command line and paste the program text into the file.

- To create the file `myprogram.py`, enter the command:

  ```bash
  nano myprogram.py
  ```
You can select any name you want, but it is not recommended to use spaces and special characters. In addition, the program extension should always end with `.py`


- Paste text in the input field. If you use Butterfly web access on Windows or Linux:

  ```
  Ctrl+Shift+V
  ```

  On Mac you can click `cmd+v`.

- Save file:

  ```
  Ctrl+x; Y; Enter
  ```

## Starting the program


- It is necessary to carefully prepare the drone, remote control and program. Run `selfcheck.py`. Make sure the drone flies in manual mode.
- Turn on the drone and wait until the system boots. A red light on the camera means that the system has booted.
- Check drone's flight in POSCTL mode
- To do this, take off above the marks in the STABILIZED mode and turn the SwC switch to the lower position - POSCTL mode.

  > **Warning** You need to be prepared to immediately switch back to STABILIZED mode if the drone gets out of control!

  Set the left stick (throttle) to the center position. The drone has to hover in place. In this case, you can land the drone and go to the next step. If not, you need to sort out the problem.

- Set the SwC switch to the middle position. It can help you to intercept the drone. You only need to switch it to the upper position.
- Set the left stick (throttle) to the middle position so that in case of interception the drone won't fall on the floor.

- Run the program:

  ```bash
  python my_program.py
  ```

  > ** Warning ** After completion of the program , the drone can land incorrectly and continue to fly over the floor. In this case, you need to intercept control.