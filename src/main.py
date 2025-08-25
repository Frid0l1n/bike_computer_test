#sensor stuff
import smbus2
import bme280
import adafruit_gps
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import RPi.GPIO as GPIO

#other stuff
import gpxpy
import time
import serial
import os
from datetime import datetime, date

#initialize display
ser = i2c(port=1, address=0x3C)
display = ssd1306(ser)
font = ImageFont.load_default()

#initialize bme
port_bme = 1
address_bme = 0x76
bus = smbus2.SMBus(port_bme)
calibration_params = bme280.load_calibration_params(bus, address_bme)

#initialize gps
uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=30)
gps_mod = adafruit_gps.GPS(uart, debug=False)
gps_mod.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')  # Output RMC and GGA sentences
gps_mod.send_command(b'PMTK220,1000')  # 1 Hz update rate

#initialize button
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
SCREEN_PIN = 16
GPIO.setup(SCREEN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
ACTIVITY_PIN = 6
GPIO.setup(ACTIVITY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def environment():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    data = bme280.sample(bus, address_bme, calibration_params)
    draw.text((0,0), "Temp: {:.2f}C".format(data.temperature), font=font, fill=25)
    draw.text((0, 16), "Press: {:.2f}hPa".format(data.pressure), font=font, fill=225)
    display.display(image)

def display_time():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    date_now = date.today()
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    draw.text((0,0), "Date: {}".format(date_now), font = font, fill=225)
    draw.text((0, 16), "Time: {}".format(current_time), font = font, fill=225)
    display.display(image)

def display_gps():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)

    gps_mod.update()
    if not gps_mod.has_fix:
        draw.text((0,0), "Waiting for signal...", font=font, fill=255)
    else:
        draw.text((0,0), "Lat: {:.4f}".format(gps_mod.latitude), font=font, fill=255)
        draw.text((0,16), "Long: {:.4f}".format(gps_mod.longitude), font=font, fill=255)
        draw.text((0, 32), "Alt: {:.1f}m".format(gps_mod.altitude_m), font=font, fill=255)

    display.display(image)


def display_activity():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    now = datetime.now()

    if not activity:
        draw.text((0,0), "No current activity running", font=font, fill=255)
    else:
        time_delta = now - activity_start
        draw.text((0,0), "Activity running:", font=font, fill=255)
        draw.text((0,16), "Time since start: {}".format(str(time_delta).split(".")[0]), font=font, fill=255)

    display.display(image)

screen_index = 0
screen_count = 4
activity = False

last_log_time = 0
log_interval = 1.0 


while True:
    if GPIO.input(16) == GPIO.LOW:
        screen_index = (screen_index + 1) % screen_count
        time.sleep(0.3)

        while GPIO.input(16) == GPIO.LOW:
            time.sleep(0.01)

    if GPIO.input(6) == GPIO.LOW:
        press_start = time.time()
        time.sleep(0.3)

        while GPIO.input(6) == GPIO.LOW:
            time.sleep(0.01)

        press_duration = time.time() - press_start

        if press_duration > 2:
            if activity:
                activity = False
                os.makedirs("gpx_logs", exist_ok = True)
                filename = "gpx_logs/activity_{}.gpx".format(datetime.now().strftime("%Y%m%d_%H%M%S"))
                with open(filename, "w") as f:
                    f.write(gpx.to_xml())
                    print(f"Saved GPX to {filename}")
        else:
            activity = True
            activity_start = datetime.now()
            #create gpx file for logging
            gpx = gpxpy.gpx.GPX()
            gpx_track = gpxpy.gpx.GPXTrack()
            gpx.tracks.append(gpx_track)
            gpx_segment = gpxpy.gpx.GPXTrackSegment()
            gpx_track.segments.append(gpx_segment)
 
    gps_mod.update()
    if activity and gps_mod.has_fix:
        now = time.time()
        if now - last_log_time > log_interval:
            lat, lon = gps_mod.latitude, gps_mod.longitude
            elev = gps_mod.altitude_m
            point = gpxpy.gpx.GPXTrackPoint(lat, lon, elevation=elev, time=datetime.now())
            gpx_segment.points.append(point)
            last_log_time = now


        
    if screen_index == 1:
        environment()
    elif screen_index == 2:
        display_time()
    elif screen_index == 3:
        display_gps()
    else:
        display_activity()

    time.sleep(0.1)

