# Working with a LED strip on Raspberry 3

## Connecting and determining the type of the strip

There are two main types of addressed LEDs: WS2812B and WS2812. The control principle is the same, but the timings are different. Locate a LED chip on the strip, and determine how many pins it has: 6 or 4. If it has 6 pins, it is WS2812; if 4, it is WS2812B or its analog SK6812.

<img src="../assets/timing_with_thumbs.png" height="400px" alt="leds">

Since the control principle is the same, the strips are connected the same way:

| strip | Raspberry Pi |
| :---: | :---: |
| GND | GND |
| 5V | 5V |
| DIN | GPIO21 or GPIO31 |

strip types for denoting the strip in the code are described in [file](https://github.com/jgarff/rpi_ws281x/blob/master/ws2811.h). The main strip types are WS2812\_STRIP \(for WS2812\) and SK6812\_STRIP \(for WS2812B or SK6812\).

## Installing a library for working with the LED strip

Define the folder that will contain files of the library, and open the path to this folder in the terminal. By default, you can use the home folder, to move to it, run command:

```(bash)
cd ~
```

Clone the repository of library for working with LED strips on Raspberry Pi:

```(bash)
git clone https://github.com/jgarff/rpi_ws281x.git
```

Install [Scons](https://ru.wikipedia.org/wiki/SCons) and [Swig](https://ru.wikipedia.org/wiki/SWIG):

```(bash)
sudo apt-get install scons python-dev swig
```

Compile the library using Scons \(the command is to be run inside the folder with the source code of the library\):

```(bash)
cd rpi_ws281x
scons
```

Compile the Python wrapper for the library using Swig, and set it for using in your Python scripts.

```(bash)
cd python
sudo python ./setup.py build
sudo python ./setup.py install
```

## Sample program for a LED bang on RPI3

In a text editor, open file `strandtest.py` from folder `python/examples` \(located in the folder with the library\):

```(bash)
nano strandtest.py
```

Locate the part of the code with strip settings:

```(bash)
# LED strip configuration:
LED_COUNT      = 16      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
LED_STRIP = ws.WS2811_STRIP_GRB # Strip type and color ordering
```

Adjust the settings for working with the strip, and save the file. To avoid interference of the use of the strip with the operation of other devices on Raspberry Pi, it is recommended to use the following settings \(the settings are for a strip together with Clever 3\):

```(bash)
# LED strip configuration:
LED_COUNT      = 30      # Number of LED pixels.
LED_PIN        = 21      # GPIO pin connected to the pixels.
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
LED_STRIP = ws.SK6812_STRIP # Strip type and color ordering
```

Run the test program from root:

```(bash)
sudo python strandtest.py
```

The root privileges are required for executing the script, since without them there is no access to the interrupt functions used by the library to work with the strip.

## Compatibility with ROS and Python

When the program is run using sudo, the user environment changes, and errors of library import appear, since required paths are missing in the environment. To add paths to Python libraries and ROS packages to the environment, add the following lines to file `/etc/sudoers` :

```(bash)
Defaults        env_keep += "PYTHONPATH"
Defaults        env_keep += "PATH"
Defaults        env_keep += "ROS_ROOT"
Defaults        env_keep += "ROS_MASTER_URI"
Defaults        env_keep += "ROS_PACKAGE_PATH"
Defaults        env_keep += "ROS_LOCATIONS"
Defaults        env_keep += "ROS_HOME"
Defaults        env_keep += "ROS_LOG_DIR"
```

## Functions for working with the LED strip

To connect the library and its correct operation, engage the following modules: neopixel - for strip operation, time – for delay management, and sys and signal for interruptions and forming the control signal.

```python
from neopixel import *
import time
import signal
import sys
```

To work with the strip, create an object of type **Adafruit_NeoPixel**, and initialize the library:

```python
# Creating a NeoPixel object with specifies configuration
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT)
# Initialization of the library, to be run before other functions
strip.begin()
```

The main functions that are used to control the strip:

* `numPixels()` - returns the number of pixels in the strip. It is convenient for cyclical control of the entire strip.
* `setPixelColor(pos, color)` – sets pixel color to position `pos`, to color `color`. The color should be a 24 bit value, where the first 8 bits are red color \(red\), the next 8 bits — green \(green\), and the last 8 bits — blue \(blue\). To get the `color` value, you can use function `Color(red, green, blue)`, which composes this value of 3 components. Each component should be in the range 0 – 255, where 0 is no color, and 255 — the highest brightness of the component available in the LED module.
* `setPixelColorRGB(pos, red, green, blue)` – sets the pixel color at position pos to the color that consists of components `red`, `green`, `blue`. Each component should be in the range 0 – 255, where 0 is no color, and 255 — the highest brightness of the component available in the LED module.
* `show()` – updates the state of the strip. It is only after using it that all software changes are moved to the LED strip.

Other functions may be detected by running command

```(bash)
pydoc neopixel
```

The result of the command:

```(bash)
    Help on module neopixel:

    NAME
        neopixel

    DESCRIPTION
        # Adafruit NeoPixel library port to the rpi_ws281x library.
        # Author: Tony DiCola (tony@tonydicola.com)

    CLASSES
        __builtin__.object
            Adafruit_NeoPixel

        class Adafruit_NeoPixel(__builtin__.object)
         |  Methods defined here:
         |
         |  __del__(self)
         |
         |  __init__(self, num, pin, freq_hz=800000, dma=5, invert=False)
         |      Class to represent a NeoPixel/WS281x LED display.  Num should be the
         |      number of pixels in the display, and pin should be the GPIO pin connected
         |      to the display signal line (must be a PWM pin like 18!).  Optional
         |      parameters are freq, the frequency of the display signal in hertz (default
         |      800khz), dma, the DMA channel to use (default 5), and invert, a boolean
         |      specifying if the signal line should be inverted (default False).
         |
         |  begin(self)
         |      Initialize library, must be called once before other functions are
         |      called.
         |
         |  getPixelColor(self, n)
         |      Get the 24-bit RGB color value for the LED at position n.
         |
         |  getPixels(self)
         |      Return an object which allows access to the LED display data as if
         |      it were a sequence of 24-bit RGB values.
         |
         |  numPixels(self)
         |      Return the number of pixels in the display.
         |
         |  setBrightness(self, brightness)
         |      Scale each LED in the buffer by the provided brightness.  The brightness
         |      of 0 is the darkest and 255 is the brightest.  Note that scaling can have
         |      quantization issues (i.e. blowing out to white or black) if used repeatedly!
         |
         |  setPixelColor(self, n, color)
         |      Set LED at position n to the provided 24-bit color value (in RGB order).
         |
         |  setPixelColorRGB(self, n, red, green, blue)
         |      Set LED at position n to the provided red, green, and blue color.
         |      Each color component should be a value from 0 to 255 (where 0 is the
         |      lowest intensity and 255 is the highest intensity).
         |
         |  show(self)
         |      Update the display with the data from the LED buffer.
         |

    FUNCTIONS
        Color(red, green, blue)
            Convert the provided red, green, blue color to a 24-bit color value.
            Each color component should be a value 0-255 where 0 is the lowest intensity
            and 255 is the highest intensity.
```

## Why exactly so, and is there a different way?

The main types of strips used for Clever 3 are WS2812, WS2812B and SK6812 \(analog to WS2812B\). They are controlled according to the same principle: for a massif of LEDs in the strip, a data packet at the rate of 24 bits per LED; each led reads the first 24 bits of received data, and sets the corresponding color, the rest of the data are sent to the next LED in the strip. Zeros and ones are determined by different combinations of the durations of high and low level in the pulse.

All these strips are supported for library management [rpi_ws281x](https://github.com/jgarff/rpi_ws281x), with that, the DMA module\(direct memory access \) of the Raspberri processor, and one of the data channels re used for control: PWM, PCM, or SPI, which guarantees absence of delays in control \(and everything is controlled from a multitasking OS, it's important\).

There are some peculiarities of working with channels, for example, when transmitting data using PWM \\ (PWM \\), the built-in audio system stops working, when transmitting data via PCM, the use of connected digital audio devices  is blocked\\ (with that, the built-in system keeps working \\), and when using SPI \(by the way, special adjustment of buffer size and Raspberry GPU frequency is required for proper operation\) the LED strip blocks all other devices connected to that channel.

There are some peculiarities of DMA channel selection to control the strip: some channels are used by the system, so using them may lead to unpleasant consequences; for example, using channel 5 destroys the Raspberry file system, since this channel is used for reading / writing to the SD card. Secure channel is No. 10, it is set by default in the library above.

Therefore, the scenarios of using the LED strip are the following:

1. If performance of embedded audio on Raspberry is not important \(and we don't use it, since the audio and the strip will produce nonsense in this case\), it is possible to use the PWM channel \(for this purpose, you have to connect strip inputs to one of the following Raspberry GPIO ports: 12, 18, 40, or 52 for PWM0 channel and 13, 19, 41, 45, or 53 for PWM1 channel\).
2. If presence of other devices on the SPI bus is not important, you can control the strip via the SPI \(GPIO on Raspberry 10 or 38\) channel.
   Here the following settings are required \(only for Raspberry Pi 3\):
   * increase the size of the data transfer buffer to support long strips, by adding line `spidev.bufsiz=32768` to file `/boot/cmdline.txt`;
   * set the GPU frequency for proper the frequency of SPI, by adding line `core_freq=250` to file `/boot/config.txt`.
3. If both audio operation and connection of SPI devices in addition to the LED strip is important, the strip may be controlled via the PCM channel \(GPIO 21 or 31\). With that, no further manipulations with Raspberry is required.

Based on the above methods of controlling the strip, the best variant that allows controlling the strip, preserve operability of the built-in audio system, and the possibility of connecting all sorts of devices and sensors via the SPI is the controlling via the PCM \(GPIO 21\) channel using the 10 DMA channel.
