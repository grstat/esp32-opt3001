# ESP32 OPT3001 LIBRARY
The OPT3001 is an Ambient Light Sensor (ALS) Manufactured by Texas Instruments that can measures lux values between 0.01 and 83k and also provides a highly configurable interrupt pin. I've tested the accuracy using my MT-912 Light Meter and the readings are pretty spot on.

This library abstracts a lot of the pain of configuring and monitoring the opt3001.

The esp32-opt1080.c provides you with a thorough example on how to use the library. In addition it gives you the information and sample code to create a an interrupt driven opt3001 setup, including an event thread, notify group and trigger setups.

# Requirements
- An ESP32 Dev Kit
- An HDC1080 i2c Sensor from Texas Instruments
- esp-idf and tools (Developed and tested on v5.1.1)

## Standard esp-idf configuration
I'd recommend the espressif IDF extension for vscode, if you haven't tried it it has come a long way and makes everything easy to setup.

### From the command line
Open the project configuration menu (`idf.py menuconfig`)
Use `idf.py -p PORT flash monitor` to build and flash to the device

### Using vscode espressif IDF extension
Command pallet -> `ESP-IDF: SDK Configuration editor`
Build the project
Command pallet -> `ESP-IDF: Build your project`
Flash it
Command pallet -> `ESP-IDF: Flash your project`
  Choose UART or JTAG depenting on your setup
  Choose the COM_PORT to use
Monitor the device
Command pallet -> `ESP-IDF: Monitor your device`

## Additional Info

Here is the Pinout used in the example

| ESP32 | OPT3001 |
|-------|---------|
| 22    | SCL     |
| 21    | SDA     |
| 33    | INT     |
| 3.3v  | VCC     |
| GND   | GND     |
| GND   | ADDR    |

You can use the header defs which are mapped to the device addresses as

|    Definition    |        Address        |
|------------------|-----------------------|
| OPT3001_ADDR_GND | 0x44 (Set in example) |
| OPT3001_ADDR_VCC | 0x45                  |
| OPT3001_ADDR_SDA | 0x46                  |
| OPT3001_ADDR_SCL | 0x47                  |

As always your feedback and requests are always welcome! Enjoy