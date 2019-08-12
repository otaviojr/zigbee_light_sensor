# Zigbee Light Sensor

This code is designed to run on a Nordic Semiconductor's nrf52840. 

Actually it was designed to run on SparkFun Pro nRF52840 Mini.
https://www.sparkfun.com/products/15025

But I'm considering to expands it to another boards in the future.

# Compiling and Flashing

1. Download Nordic nRF5 SDK (https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK-for-Thread-and-Zigbee)
2. Create a projects folder inside SDK. At the same level that the example folder
3. Clone this repo inside the projects folder
```
$ cd <nRF root>/projects
$ git clone https://github.com/otaviojr/zigbee_light_sensor.git .
$ cd zigbee_light_sensor/sparkfun/blank/armgcc
$ make
```
4. To flash, click twice the reset button and than

```
$ make bootload SERIAL_PORT=/dev/ttyACM0 
```
