# Clever 4 assembly

## Frame base assembly

1. Align two reinforcement carbon plates using the center notch.
2. Install the top carbon deck using notches as guides.

    <div class="image-group">
        <img src="../assets/4/01.png" width=300 class="zoom border">
        <img src="../assets/4/02.png" width=300 class="zoom border">
    </div>

3. Place self-locking steel nuts into the slots in reinforcement plates and tighten the assembly with M3x8 screws.

    <div class="image-group">
        <img src="../assets/4/03.png" width=300 class="zoom border">
        <img src="../assets/4/03_1.png" width=300 class="zoom border">
    </div>

## Installing motors

1. Unbox the motors.
2. Shorten the motor wires using wire strippers or sidecutters:

    * Cut wires to 30 mm.
    * Strip 5 mm of insulation while taking care to not damage the cores

    <img src="../assets/4/04.png" width=300 class="zoom center border">

    * Twist the cores.
    * [Tin the wires](tinning.md). You may want to use tweezers to hold the wire.

3. Place the motor on the support arm.
4. Use hexagonal M3x5 screws to attach the motor to its arm.

<img src="../assets/4/05.png" width=300 class="zoom center border">

Perform these actions for each motor.

## Frame assembly

1. Install the support arms on the frame base according to their rotation direction. Use notches as guides.

    <img src="../assets/4/05.png" width=300 class="zoom center border">

    > **Hint** Note the motor nut colors when installing the arms. Motors with red nuts should be placed on the front right and back left arms, with black ones - on the front left and back right arms.

2. Attach the arms to the frame base using 8 M3x8 screws, 6 steel nuts, and 2 15 mm spacers.

    <div class="image-group">
        <img src="../assets/4/07.png" width=300 class="zoom border">
        <img src="../assets/4/07_1.png" width=300 class="zoom border">
    </div>

## Preparing the power distribution board

1. [Tin](tinning.md) the pads on the power distribution board.
2. Check the board for shorts using a multimeter:
    * Set your multimeter to the Continuity Test mode.
    * Ensure your multimeter works by connecting the probes to each other. The multimeter should beep.
    * Connect one of the probes to the **«+»** pad and the other to **«-»/GND**. If there is a short circuit, the multimeter will beep.

## Mounting the PDB

1. Attach four 6 mm standoffs on the top carbon deck using M3x6 screws.

    <div class="image-group">
        <img src="../assets/4/08.png" width=300 class="zoom border">
        <img src="../assets/4/08_1.png" width=300 class="zoom border">
    </div>

2. Place the PDB on the standoffs.
3. Make sure the arrow on the PDB is pointing in the same direction as the arrow on the top carbon deck.

    <img src="../assets/4/09.png" width=300 class="zoom center border">

## Soldering the speed controllers and the BEC

1. Solder the motor wires to the electronic speed controllers (ESCs).
2. Solder the ESC power wires to the power distribution board (**<font color=red>red</font>** to **«+»**, **black** to **«-»**).

    <div class="image-group">
        <img src="../assets/4/09.png" width=300 class="zoom border">
        <img src="../assets/4/10_1.png" width=300 class="zoom border">
    </div>

3. Solder power wires of the battery elimination circuit in parallel to one of the ESC power wires (**<font color=red>red</font>** to **«+»**, **black** to **«-»**).

    <div class="image-group">
        <img src="../assets/4/10.png" width=300 class="zoom border">
        <img src="../assets/4/12_1.png" width=300 class="zoom border">
    </div>

4. Check the board for shorts using a multimeter.

## Checking motor rotation direction

Motors with **<font color=red>red</font>** nuts should rotate **counterclockwise**, the ones with **black** nuts should rotate **clockwise**. Correct rotation direction should also be printed on the motors. You can use a servo tester or your RC transmitter and receiver to check rotation direction.

<img src="../assets/4/props/props_rotation.png" width=300 class="zoom center">

The following describes how to use your RC gear to check the motor direction.

### Setting up PWM mode on RC

Turn on your transmitter using the **POWER** slider. If the RC transmitter is locked, place all controls in their neutral position:

1. Left stick should be in the **lower center position**.
2. Right stick should be **centered**.
3. The switches (A, B, C, D) should be in the **top position**.

    <img src="../assets/4/radio/base.png" alt="Neutral control state" width=500 class="zoom center">

Make sure the transmitter operates in the PWM mode:

1. Power down the receiver.
2. Hold down the "OK" button to enter the menu.
3. Select the "System setup" option, press "OK" to enter the submenu.
4. Select "RX Setup" option.
5. Select "Output mode".
6. Make sure the "PWM" option is selected.
7. Save settings by holding the "Cancel" button.

### Binding the RC transmitter and receiver

1. Turn off the RC transmitter with the **POWER** slider.
2. Connect the RC receiver to the 5 V BEC output. Connect the black wire into one of the bottom pins and the red wire to one of the central pins.
3. Place the binding jumper on the B/VCC output.
4. Connect the battery pack.
5. The LED on the RC receiver should start to blink.

    <div class="image-group">
        <img src="../assets/4/13.png" width=300 class="zoom border">
        <img src="../assets/4/13_1.png" width=300 class="zoom border">
    </div>

6. Hold down the **BIND KEY** on the RC transmitter.
7. Turn on the RC transmitter while holding the **BIND KEY**

    <img src="../assets/4/radio/binding.png" width=500 class="zoom center">

8. Wait for the **RXBind ok** message on the RC transmitter
9. Disconnect the binding jumper.
10. The LED on the RC receiver should be lit continuously.

### Checking the motor rotation direction

1. Disconnect the battery pack and power down the transmitter.
2. Connect the signal wires from the ESC to CH3 pins on the output. The white wire should go to the top pin, the black one should go to the bottom one.
3. Power on the transmitter. Make sure the left stick is in the bottom position.
4. Connect the battery pack.
5. Slowly move the left stick up until the motor starts to spin.

If the motor rotation direction is wrong, switch any two motor wires.

> **Info** You can also change motor direction by reprogramming the speed controllers. The process is described [in the ESC firmware flashing article](esc_firmware.md).

Do this for each motor.

### Switching the transmitter back to PPM mode

The flight controller expects PPM signal from your RC gear. Switch your transmitter back to PPM before flight.

1. Make sure the receiver is not powered.
2. Hold down the "OK" button to enter the menu.
3. Select the "System setup" option, press "OK" to enter the submenu.
4. Select "RX Setup" option.
5. Select "Output mode".
6. Make sure the "PPM" option is selected.
7. Save settings by holding the "Cancel" button.

## Mounting the flight controller plate

1. Attach four 6 mm standoffs on top of PDB.
2. Connect the flight controller power cable to the PDB.
3. Place the polycarbonate plate on the standoffs and fix them with plastic nuts.

    <div class="image-group">
        <img src="../assets/4/14.png" width=300 class="zoom border">
        <img src="../assets/4/15.png" width=300 class="zoom border">
        <img src="../assets/4/16.png" width=300 class="zoom border">
    </div>

## Mounting the flight controller

1. Insert the microSD card into your flight controller.

    <img src="../assets/4/pix/pixracer_sdcard.png" width=300 class="zoom center border">

2. Align the flight controller so that the arrows on the controller and on the top carbon deck point in the same direction.
3. Attach the flight controller to the flight controller plate using 3M double-sided adhesive pads.
4. Connect the power cable to the **"POWER"** input of the flight controller.

    <div class="image-group">
        <img src="../assets/4/18.png" width=300 class="zoom border">
        <img src="../assets/4/18_1.png" width=300 class="zoom border">
    </div>

5. Attach four 40 mm aluminum spacers to the top carbon deck using M3x10 screws.

    <div class="image-group">
        <img src="../assets/4/19.png" width=300 class="zoom border">
        <img src="../assets/4/19_1.png" width=300 class="zoom border">
    </div>

6. Connect signal wires to the flight controller as shown in these pictures:

    <div class="image-group">
        <img src="../assets/4/20.png" width=300 class="zoom border">
        <img src="../assets/4/20_1.png" width=300 class="zoom border">
    </div>

7. Attach two 15 mm spacers to the top carbon deck using M3x8 screws.
8. Attach two 15 mm spacers to the top carbon deck and the front arms using M3x10 screws (this was already described in the "Frame Assembly" section, p. 2).

    <div class="image-group">
        <img src="../assets/4/21.png" width=300 class="zoom border">
        <img src="../assets/4/21_1.png" width=300 class="zoom border">
    </div>

## Mounting the LED strip ring

1. Bend the polycarbonate strip into a ring and use the locks to fix it in this shape.
2. Fix the ring on the frame using appropriate notches.

    <div class="image-group">
        <img src="../assets/4/22.png" width=300 class="zoom border">
        <img src="../assets/4/23.png" width=300 class="zoom border">
    </div>

## Installing the Raspberry Pi

1. Insert your microSD card [with our image](image.md) into the Raspberry Pi

    <img src="../assets/4/rpi/rpi_sdcard.png" width=300 class="zoom center border">

2. Attach the Raspberry Pi using four standoffs.
3. Route the BEC wires through the channel in the top carbon deck.

    <div class="image-group">
        <img src="../assets/4/24.png" width=300 class="zoom border">
        <img src="../assets/4/26.png" width=300 class="zoom border">
    </div>

4. Connect the BEC outputs according to the following image:

<img src="../assets/4/26_1.png" width=300 class="zoom center border">

## Installing the LED strip on the LED strip ring

1. Check wires on the strip (and solder them on if they're missing)
2. Attach the LED strip to the ring using the adhesive layer on the strip. Use zip ties to fix it in place.

    <div class="image-group">
        <img src="../assets/4/27.png" width=300 class="zoom border">
        <img src="../assets/4/27_1.png" width=300 class="zoom border">
        <img src="../assets/4/28.png" width=300 class="zoom border">
    </div>

## Connecting the LED strip to Raspberry Pi

1. Power the LED strip from a separate BEC. Connect the **«+»** and **«-»** leads to **5v** and **Ground** respectively.
2. Connect the **D** lead to GPIO21 (consult the [relevant article](leds.md) for more information).

    <div class="image-group">
        <img src="../assets/4/31_1.png" width=300 class="zoom border">
        <img src="../assets/4/31_2.png" width=300 class="zoom border">
    </div>

## Installing the camera cable

1. Open the slot connector by lifting the T-clip.
2. Insert the ribbon cable.
3. Press the T-clip down to secure the cable.

<img src="../assets/4/32.png" width=300 class="zoom center border">

## Mounting the lower deck periphery

1. Prepare the laser rangefinder by soldering leads to it.
2. Use four 2x5 self-tapping screws to secure the camera.
3. Mount the laser rangefinder on the lower deck using two M3x8 screws and steel nuts.

    <div class="image-group">
        <img src="../assets/4/33.png" width=300 class="zoom border">
        <img src="../assets/4/33_1.png" width=300 class="zoom border">
    </div>

4. Attach RC receiver to the lower deck using 3M double-sided adhesive pads.

    <img src="../assets/4/34.png" width=300 class="zoom center border">

5. Mount the lower deck assembly using four M3x10 screws.
6. Connect the camera ribbon cable to the camera.

    <div class="image-group">
        <img src="../assets/4/35.png" width=300 class="zoom border">
        <img src="../assets/4/36.png" width=300 class="zoom border">
    </div>

7. Connect the laser rangefineder to the Raspberry Pi using the following pinout:
    * Connect **VCC** to pin 1 (**3v3**)
    * Connect **GND** to pin 9 (**Ground**)
    * Connect **SDA** to pin 3 (**GPIO2**)
    * Connect **SCL** to pin 5 (**GPIO3**)

    <img src="../assets/4/36.png" width=300 class="zoom center border">

## Mounting the landing gear

1. Attach 8 landing gear pieces using M3x10 screws and steel nuts.
2. Attach dampening pads to the landing gear pieces using M3x10 screws and steel nuts.

    <div class="image-group">
        <img src="../assets/4/37.png" width=300 class="zoom border">
        <img src="../assets/4/38.png" width=300 class="zoom border">
    </div>

## Connecting the cables

1. Connect RC cable to the **RCIN** port on the flight controller.
2. Connect RC cable to RC receiver.

    <img src="../assets/4/39.png" width=300 class="zoom center border">

## Mounting the propeller guards

1. Assemble the lower part of the guards using twelve M3x10 screws and twelve 40 mm plastic spacers.
2. Assemble the top part using twelve M3x10 screws.

    <div class="image-group">
        <img src="../assets/4/40.png" width=300 class="zoom border">
        <img src="../assets/4/41.png" width=300 class="zoom border">
    </div>

3. Attach the assembly to the drone using four M3x10 screws and steel nuts.

    <div class="image-group">
        <img src="../assets/4/42.png" width=300 class="zoom border">
        <img src="../assets/4/42_1.png" width=300 class="zoom border">
    </div>

## Mounting the top deck

1. Attach the battery holder to the top deck with four M3x8 screws and steel nuts.
2. Thread the battery strap through the slots in the deck.
3. Attach the top deck using four M3x10 screws.

    <div class="image-group">
        <img src="../assets/4/42.png" width=300 class="zoom border">
        <img src="../assets/4/42_1.png" width=300 class="zoom border">
    </div>

4. Connect the flight controller to the Raspberry Pi using retractable USB cable.

    <img src="../assets/4/43_1.png" width=300 class="zoom center border">

5. Attach the USB cable reel where convenient using 3M double-sided adhesive pads while making sure the cable does not interfere with the propellers.

    <div class="image-group">
        <img src="../assets/4/47_1.png" width=300 class="zoom border">
        <img src="../assets/4/47_2.png" width=300 class="zoom border">
    </div>

## Installing the battery

> **Warning** Make sure all cables are secured and nothing interferes with the propellers!

Check the quadrotor assembly:

* The balance connector should be fixed under the battery strap.
* The ESCs should be zip tied to the frame.
* All wires from the PDB and flight controller should be tucked under a velcro strap wound around aluminum spacers.

<img src="../assets/4/48.png" width=300 class="zoom center border">

## Mounting the propellers and preparing for flight

Perform the quadrotor components setup according to [the "Configuration" section](setup.md).

> **Warning** Be sure to **not** mount the propellers **until the setup is complete**. Do it only when you are ready to fly.

Attach the propellers according to their rotation direction. The battery should be disconnected duting propeller installation.

<div class="image-group">
    <img src="../assets/4/49.png" width=300 class="zoom border">
    <img src="../assets/4/50.png" width=300 class="zoom border">
</div>

> **Success** The drone is ready to fly!
