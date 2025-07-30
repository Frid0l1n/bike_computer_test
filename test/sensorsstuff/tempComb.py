import smbus2
import bme280
import time
#stuff for the other shit
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

port = 1
address = 0x76
bus = smbus2.SMBus(port)

calibration_params = bme280.load_calibration_params(bus, address)
serial = i2c(port=1, address=0x3C)
display = ssd1306(serial)

font = ImageFont.load_default()



while True:
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    data = bme280.sample(bus, address, calibration_params)
    draw.text((0,0), "Temp: {:.2f}C".format(data.temperature), font=font, fill=25)
    draw.text((0, 16), "Press: {:.2f}hPa".format(data.pressure), font=font, fill=225)
    display.display(image)
    time.sleep(2)

