# sensor stuff
import smbus2
import bme280
import adafruit_gps
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import RPi.GPIO as GPIO

# other stuff
import gpxpy
import xml.etree.ElementTree as ET
import time
import serial
import os
from datetime import datetime, date

# initialize display
ser = i2c(port=1, address=0x3C)
display = ssd1306(ser)
font = ImageFont.load_default()

# initialize bme
port_bme = 1
address_bme = 0x76
bus_bme = smbus2.SMBus(port_bme)
calibration_params = bme280.load_calibration_params(bus_bme, address_bme)

# initialize icm
port_icm = 1
address_icm = 0x68 

REG_BANK_SEL = 0x7F
REG_PWR_MGMT_1 = 0x06
REG_ACCEL_XOUT_H = 0x2D
REG_ACCEL_CONFIG = 0x14

bus_icm = smbus2.SMBus(port_icm)

def switch_bank(bank):
    bus_icm.write_byte_data(address_icm, REG_BANK_SEL, bank << 4)

def initialize_icm():
    switch_bank(0)
    bus_icm.write_byte_data(address_icm, REG_PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    switch_bank(2)
    bus_icm.write_byte_data(address_icm, REG_ACCEL_CONFIG, 0x00)
    time.sleep(0.1)
    switch_bank(0)

def read_accel():
    data = bus_icm.read_i2c_block_data(address_icm, REG_ACCEL_XOUT_H, 6)

    x = (data[0] << 8) | data[1]
    y = (data[2] << 8) | data[3]
    z = (data[4] << 8) | data[5]
    
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z

    ax = x / 16384.0
    ay = y / 16384.0
    az = z / 16384.0
    
    return ax, ay, az

initialize_icm()

# initialize gps
uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=30)
gps_mod = adafruit_gps.GPS(uart, debug=False)
gps_mod.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')  # Output RMC and GGA sentences
gps_mod.send_command(b'PMTK220,1000')  # 1 Hz update rate

# initialize button
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
SCREEN_PIN = 16
GPIO.setup(SCREEN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
ACTIVITY_PIN = 6
GPIO.setup(ACTIVITY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def environment():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    data = bme280.sample(bus_bme, address_bme, calibration_params)
    draw.text((0, 0), "Temp: {:.2f}C".format(data.temperature), font=font, fill=255)
    draw.text((0, 16), "Press: {:.2f}hPa".format(data.pressure), font=font, fill=255)
    display.display(image)

def display_time():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    date_now = date.today()
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    draw.text((0, 0), "Date: {}".format(date_now), font=font, fill=255)
    draw.text((0, 16), "Time: {}".format(current_time), font=font, fill=255)
    display.display(image)

def display_gps():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)

    gps_mod.update()
    if not gps_mod.has_fix:
        draw.text((0, 0), "Waiting for signal...", font=font, fill=255)
    else:
        draw.text((0, 0), "Lat: {:.4f}".format(gps_mod.latitude), font=font, fill=255)
        draw.text((0, 16), "Long: {:.4f}".format(gps_mod.longitude), font=font, fill=255)
        draw.text((0, 32), "Alt: {:.1f}m".format(gps_mod.altitude_m), font=font, fill=255)

    display.display(image)

def display_activity():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    now = datetime.now()

    if not activity:
        draw.text((0, 0), "No current activity running", font=font, fill=255)
    else:
        time_delta = now - activity_start
        draw.text((0, 0), "Activity running:", font=font, fill=255)
        draw.text((0, 16), "Time since start: {}".format(str(time_delta).split(".")[0]), font=font, fill=255)

    display.display(image)


def display_accelerometer(ax, ay, az):
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    draw.text((0, 0), "AX{:.2f}".format(ax), font=font, fill=255)
    draw.text((0, 16), "AZ{:.2f}".format(az), font=font, fill=255)
    draw.text((0, 32), "AY{:.2f}".format(ay), font=font, fill=255)

    display.display(image)


def log_activity(activity):
    name = datetime.now() 
    clean_name = name.strftime("%Y-%m-%d_%H-%M")
    
    gpx = ET.Element("gpx", version="1.1", creatpor="MyTest")
    trk = ET.SubElement(gpx, "trk")
    trkseg = ET.SubElement(trk, "trkseg")


    while activity:
         gps_mod.update()
         lat = gps_mod.latitude
         long = gps_mod.longitude
         alt = gps_mod.altitude_m

         trkpt = ET.SubElement(trkseg, "trkpt", lat=str(lat), lon=str(long))
         ele = ET.SubElement(trkpt, "ele")
         ele.text = str(alt)

         time = ET.SubElement(trkpt, "time")
         time.text = datetime.datetime.now().isoformat()

    tree = ET.ElementTree(gpx)

    with open(f"{clean_name}.gpx", "wb") as f:
        tree.write(f)


# state vars
screen_index = 0
screen_count = 5
activity = False

last_log_time = 0
log_interval = 1.0

while True:

    # screen switch button
    if GPIO.input(SCREEN_PIN) == GPIO.LOW:
        screen_index = (screen_index + 1) % screen_count
        time.sleep(0.3)
        while GPIO.input(SCREEN_PIN) == GPIO.LOW:
            time.sleep(0.01)

    # activity button
    if GPIO.input(ACTIVITY_PIN) == GPIO.LOW:
        press_start = time.time()
        time.sleep(0.3)
        while GPIO.input(ACTIVITY_PIN) == GPIO.LOW:
            time.sleep(0.01)

        press_duration = time.time() - press_start

        if press_duration > 2:
            # Long press → stop activity
            if activity:
                activity = False
                os.makedirs("gpx_logs", exist_ok=True)
                filename = "gpx_logs/activity_{}.gpx".format(datetime.now().strftime("%Y%m%d_%H%M%S"))
                with open(filename, "w") as f:
                    f.write(gpx.to_xml())
                    print(f"Saved GPX to {filename}")
            else:
                print("No activity to stop.")
        else:
            # Short press → start activity
            if not activity:
                activity = True 
                activity_start = datetime.now()
                log_activity(activity)
                print("Activity started.")
            else:
                print("Activity already running.")

    # log GPS points if activity running
    if activity and gps_mod.has_fix:
        now = time.time()
        if now - last_log_time > log_interval:
            lat, lon = gps_mod.latitude, gps_mod.longitude
            elev = gps_mod.altitude_m
            point = gpxpy.gpx.GPXTrackPoint(
                lat, lon, elevation=elev, time=datetime.now()
            )
            gpx_segment.points.append(point)
            last_log_time = now


    ax, ay, az = read_accel()

    # screen cycling
    if screen_index == 1:
        environment()
    elif screen_index == 2:
        display_time()
    elif screen_index == 3:
        display_gps()
    elif screen_index == 4:
        display_accelerometer(ax, ay, az)
    else:
        display_activity()

    time.sleep(0.1)
