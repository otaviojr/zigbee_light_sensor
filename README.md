# Zigbee Light Sensor

This code is designed to run on a Nordic Semiconductor's nrf52840. 

Actually it was designed to run on SparkFun Pro nRF52840 Mini.\
https://www.sparkfun.com/products/15025

I'm considering to expands it to another boards in the future.

Anyway, porting it to any other nrf52840 board should not be a big deal.

## Materials

1. [SparkFun Pro nRF52840 mini](https://www.sparkfun.com/products/15025?_ga=2.111855680.592339865.1564452186-1575453690.1551457345)
2. [SeeedStudio Digital Light Sensor](https://www.seeedstudio.io/s/Grove-Digital-Light-Sensor-p-1281.html)
3. [Qwiic Grove Cable Adapter](https://www.sparkfun.com/products/15109)

## Compiling and Flashing

1. Download Nordic nRF5 SDK\
https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK-for-Thread-and-Zigbee
2. Create a projects folder inside the SDK root dir, at the same level that the example folder is.
3. Clone this repo inside the projects folder we just created
```
$ cd <nRF SDK root>/projects
$ git clone https://github.com/otaviojr/zigbee_light_sensor.git .
$ cd zigbee_light_sensor/sparkfun/blank/armgcc
$ make
```
4. To flash, click twice the reset button and than

```
$ make bootload SERIAL_PORT=/dev/ttyACM0 
```
