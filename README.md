# Zigbee Light Sensor

This code is designed to run on a Nordic Semiconductor's nrf52840.

Actually it was designed to run on SparkFun Pro nRF52840 Mini.

I'm considering to extends it to another boards in the future.

Anyway, porting it to any other nrf52840 board should not be a big deal.

## Materials

1. [SparkFun Pro nRF52840 mini](https://www.sparkfun.com/products/15025?_ga=2.111855680.592339865.1564452186-1575453690.1551457345)
2. [SeeedStudio Digital Light Sensor](https://www.seeedstudio.io/s/Grove-Digital-Light-Sensor-p-1281.html)
3. [Qwiic Grove Cable Adapter](https://www.sparkfun.com/products/15109)

## Preparing

1. Download and Install nrfutil (linux)

## Compiling and Flashing

1. Download Nordic nRF5 SDK\
https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK-for-Thread-and-Zigbee
2. Patching Nordic nRF5 SDK to support sparkfun board<br>
	Inside this github project you will find a folder named "patch" with two files:<br>
	1. The boards.h.patch is a patch to the original boards.h located at `<nRF SDK root>/components/boards`.
	2. The sparkfun_nrf52840_mini.h file must be copyed to the same folter.
3. Create a projects folder inside the SDK root dir, at the same level that the example folder is.
```
$ mkdir <nRF SDK root>/projects
```
4. Clone this repo inside the projects folder we just created
```
$ cd <nRF SDK root>/projects
$ git clone https://github.com/otaviojr/zigbee_light_sensor.git .
$ cd zigbee_light_sensor/sparkfun/blank/armgcc
$ make
```
5. To flash, click the reset button twice and than:

```
$ make bootload SERIAL_PORT=/dev/ttyACM0
```
## LEDs

Since SparkFun Pro nRF52840 Mini has only one LED I will try to use it for everything.

Initialy the blue LED will blink fast to indicate that the node is trying to join a network. After the node joins a network the blue LED will blink slowly.

## Buttons

Since SparkFun Pro nRF52840 Mini has only one button, again, I will need to use it for everything.

The number of clicks will define which action have to be executed.

* 2 clicks - Change light sensor sensitivity\
The LED will blink 3 times faster, then it will blink 1, 2 or 3 times slow, this is the sensitivity level, then, it will blink more 3 times faster.
* 3 clicks - Blink the LED to show the current light sensor sensitivity\
The LED will blink 3 times faster, then it will blink 1, 2 or 3 times slow, this is the sensitivity level, then, it will blink more 3 times faster.
* 10 clicks - Factory Reset. Allow the sensor to join on a new network

## Light Sensor Sensitivity Level

1. Low Sensitivity\
(1x/402ms)
2. Medium Sensitivity\
**Is not working the way I want it to work, use the high and  low  sensitivity for now**\
(16x/101ms)
3. High Sensitivity\
(16x/402ms)
