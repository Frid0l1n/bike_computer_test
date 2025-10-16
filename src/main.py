import smbus2
import bme280
import pynmea2
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import RPi.GPIO as GPIO
import gpxpy
import gpxpy.gpx
import time
import serial
import os
from datetime import datetime
from collections import deque

# Display initialization
ser = i2c(port=1, address=0x3C)
display = ssd1306(ser)
font = ImageFont.load_default()

# BME280 initialization
port_bme = 1
address_bme = 0x76
bus_bme = smbus2.SMBus(port_bme)
calibration_params = bme280.load_calibration_params(bus_bme, address_bme)

# ICM-20948 initialization
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

# GPS initialization
uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

# GPIO setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
SCREEN_PIN = 16
ACTIVITY_PIN = 6
GPIO.setup(SCREEN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ACTIVITY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# GPS data class
class GPSData:
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed_knots = None
        self.speed_kmh = None
        self.has_fix = False
        self.satellites = 0
        self.timestamp = None
        
    def update_from_nmea(self, sentence):
        try:
            msg = pynmea2.parse(sentence)
            
            if isinstance(msg, pynmea2.types.talker.GGA):
                self.latitude = msg.latitude
                self.longitude = msg.longitude
                self.altitude = msg.altitude if msg.altitude else 0
                self.has_fix = msg.gps_qual > 0
                self.satellites = msg.num_sats if msg.num_sats else 0
                
            elif isinstance(msg, pynmea2.types.talker.RMC):
                if msg.latitude and msg.longitude:
                    self.latitude = msg.latitude
                    self.longitude = msg.longitude
                if msg.spd_over_grnd:
                    self.speed_knots = float(msg.spd_over_grnd)
                    self.speed_kmh = self.speed_knots * 1.852
                self.timestamp = msg.timestamp
                
        except (pynmea2.ParseError, AttributeError):
            pass

gps_data = GPSData()

# Activity tracking class
class ActivityTracker:
    def __init__(self):
        self.active = False
        self.start_time = None
        self.gpx = None
        self.gpx_segment = None
        self.speed_buffer = deque(maxlen=30)  # Last 30 speed readings
        self.avg_speed = 0.0
        self.max_speed = 0.0
        self.point_count = 0
        
    def start(self):
        self.active = True
        self.start_time = datetime.now()
        self.gpx = gpxpy.gpx.GPX()
        self.gpx.creator = "RPi Activity Tracker"
        gpx_track = gpxpy.gpx.GPXTrack()
        self.gpx.tracks.append(gpx_track)
        self.gpx_segment = gpxpy.gpx.GPXTrackSegment()
        gpx_track.segments.append(self.gpx_segment)
        self.speed_buffer.clear()
        self.avg_speed = 0.0
        self.max_speed = 0.0
        self.point_count = 0
        print("Activity started")
        
    def stop(self):
        if not self.active:
            return None
            
        self.active = False
        os.makedirs("gpx_logs", exist_ok=True)
        filename = "gpx_logs/activity_{}.gpx".format(
            self.start_time.strftime("%Y%m%d_%H%M%S")
        )
        
        with open(filename, "w") as f:
            f.write(self.gpx.to_xml())
        
        print(f"Activity stopped. Saved to {filename}")
        print(f"Points logged: {self.point_count}")
        print(f"Average speed: {self.avg_speed:.2f} km/h")
        print(f"Max speed: {self.max_speed:.2f} km/h")
        return filename
        
    def add_point(self, lat, lon, elev, speed_kmh=None):
        if not self.active:
            return
            
        point = gpxpy.gpx.GPXTrackPoint(
            latitude=lat,
            longitude=lon,
            elevation=elev,
            time=datetime.utcnow()
        )
        self.gpx_segment.points.append(point)
        self.point_count += 1
        
        # Update speed statistics
        if speed_kmh is not None and speed_kmh > 0:
            self.speed_buffer.append(speed_kmh)
            self.avg_speed = sum(self.speed_buffer) / len(self.speed_buffer)
            self.max_speed = max(self.max_speed, speed_kmh)
    
    def get_duration(self):
        if not self.active or not self.start_time:
            return "00:00:00"
        delta = datetime.now() - self.start_time
        hours, remainder = divmod(int(delta.total_seconds()), 3600)
        minutes, seconds = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

activity_tracker = ActivityTracker()

# Display functions
def environment():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    data = bme280.sample(bus_bme, address_bme, calibration_params)
    draw.text((0, 0), f"Temp: {data.temperature:.1f}C", font=font, fill=255)
    draw.text((0, 16), f"Press: {data.pressure:.1f}hPa", font=font, fill=255)
    draw.text((0, 32), f"Hum: {data.humidity:.1f}%", font=font, fill=255)
    display.display(image)

def display_time():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    now = datetime.now()
    draw.text((0, 0), now.strftime("%Y-%m-%d"), font=font, fill=255)
    draw.text((0, 16), now.strftime("%H:%M:%S"), font=font, fill=255)
    display.display(image)

def display_gps():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)

    if not gps_data.has_fix:
        draw.text((0, 0), "Waiting for GPS...", font=font, fill=255)
        draw.text((0, 16), f"Sats: {gps_data.satellites}", font=font, fill=255)
    else:
        draw.text((0, 0), f"Lat: {gps_data.latitude:.4f}", font=font, fill=255)
        draw.text((0, 16), f"Lon: {gps_data.longitude:.4f}", font=font, fill=255)
        draw.text((0, 32), f"Alt: {gps_data.altitude:.1f}m", font=font, fill=255)
        if gps_data.speed_kmh is not None:
            draw.text((0, 48), f"Spd: {gps_data.speed_kmh:.1f}km/h", font=font, fill=255)

    display.display(image)

def display_activity():
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)

    if not activity_tracker.active:
        draw.text((0, 0), "No activity", font=font, fill=255)
        draw.text((0, 16), "Short press: START", font=font, fill=255)
    else:
        draw.text((0, 0), f"Time: {activity_tracker.get_duration()}", font=font, fill=255)
        draw.text((0, 16), f"Avg: {activity_tracker.avg_speed:.1f}km/h", font=font, fill=255)
        draw.text((0, 32), f"Max: {activity_tracker.max_speed:.1f}km/h", font=font, fill=255)
        draw.text((0, 48), f"Pts: {activity_tracker.point_count}", font=font, fill=255)

    display.display(image)

def display_accelerometer(ax, ay, az):
    image = Image.new("1", display.size)
    draw = ImageDraw.Draw(image)
    draw.text((0, 0), f"X: {ax:.2f}g", font=font, fill=255)
    draw.text((0, 16), f"Y: {ay:.2f}g", font=font, fill=255)
    draw.text((0, 32), f"Z: {az:.2f}g", font=font, fill=255)
    display.display(image)

# State variables
screen_index = 0
screen_count = 5
last_log_time = 0
log_interval = 2.0  # Log every 2 seconds

print("Activity tracker started. Use buttons to navigate.")

try:
    while True:
        # Read GPS data
        if uart.in_waiting > 0:
            try:
                line = uart.readline().decode('ascii', errors='replace').strip()
                gps_data.update_from_nmea(line)
            except (UnicodeDecodeError, serial.SerialException):
                pass

        # Screen switch button
        if GPIO.input(SCREEN_PIN) == GPIO.LOW:
            screen_index = (screen_index + 1) % screen_count
            time.sleep(0.3)
            while GPIO.input(SCREEN_PIN) == GPIO.LOW:
                time.sleep(0.01)

        # Activity button
        if GPIO.input(ACTIVITY_PIN) == GPIO.LOW:
            press_start = time.time()
            time.sleep(0.3)
            while GPIO.input(ACTIVITY_PIN) == GPIO.LOW:
                time.sleep(0.01)

            press_duration = time.time() - press_start

            if press_duration > 2:
                # Long press → stop activity
                activity_tracker.stop()
            else:
                # Short press → start activity
                if not activity_tracker.active:
                    activity_tracker.start()

        # Log GPS points if activity is running
        if activity_tracker.active and gps_data.has_fix:
            now = time.time()
            if now - last_log_time >= log_interval:
                activity_tracker.add_point(
                    gps_data.latitude,
                    gps_data.longitude,
                    gps_data.altitude if gps_data.altitude else 0,
                    gps_data.speed_kmh
                )
                last_log_time = now

        # Read accelerometer
        ax, ay, az = read_accel()

        # Display current screen
        if screen_index == 0:
            display_activity()
        elif screen_index == 1:
            environment()
        elif screen_index == 2:
            display_time()
        elif screen_index == 3:
            display_gps()
        elif screen_index == 4:
            display_accelerometer(ax, ay, az)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nShutting down...")
    if activity_tracker.active:
        activity_tracker.stop()
    GPIO.cleanup()
    uart.close()
