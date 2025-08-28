# Bike Computer Test

This project aims to create a simple open-source bike computer, similar to models from Garmin or Wahoo. We are currently in the very early stages, experimenting with breadboards and writing Python code to implement the basic functionalities.

## Table of Contents
- [Project Setup](#project-setup)  
- [Hardware Used](#hardware-used)  
- [Required Packages](#required-packages)  
- [Use It On Your Own](#use-it-on-your-own)  
- [Found a Bug?](#found-a-bug)

## Project Setup
1. Clone this repository.  
2. Set up a Python [virtual environment](https://docs.python.org/3/tutorial/venv.html) and activate it.  
3. Install the required packages.  
4. Run the main file in the `src` directory.  
5. Have fun!

## Hardware Used

| Component | Price |
| --------- | ----- |
| [Raspberry Pi Zero](https://eu.mouser.com/ProductDetail/358-SC0065) | 12.95 CHF |
| [PIM448 Motion Sensor](https://eu.mouser.com/ProductDetail/397-PIM448) | 14.95 CHF |
| [BME280 Sensor](https://eu.mouser.com/ProductDetail/Pimoroni/PIM472?qs=P1JMDcb91o7p2TYl00AP7g%3D%3D) | 13.80 CHF |
| [I2C OLED Display 0.96"](https://eu.mouser.com/ProductDetail/Soldered/333099?qs=sGAEpiMZZMu3sxpa5v1qrkxCKRcLLrCR550%252BadnWsO8%3D) | 12.05 CHF |
| [B3F-4130 Button](https://eu.mouser.com/ProductDetail/653-B3F-4150) | 0.55 CHF per piece |
| [Breadboard](https://eu.mouser.com/ProductDetail/BusBoard-Prototype-Systems/BB830T?qs=VEfmQw3KOauXY1NKV2FuEg%3D%3D) | 8–10 CHF |

**Total estimated cost:** ~66 CHF

## Required Packages

Make sure to install the following Python packages:

## Required Packages

```
smbus2
icm20948-python
bme280
adafruit_gps
luma.oled
RPi.GPIO
csv
time
datetime
gpxpy
```

> You can use a `requirements.txt` file to make installation easier.

## Own use cases
Since this is a big project, i'd encourage you to clone rename it and use it for your own puproses. I guess it is enough for a solid start into the field of diy bike computers.
## Found a bug
If you found an issue or would like to submit code improvements or other stuff to this project, please submitt the issue using the issue tab above. If you would like to submitt a PR with a fix, reference the issue you created!
