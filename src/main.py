import smbus2
import bme280
import pynmea2
import datetime
import time
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import RPi.GPIO as GPIO
import gpxpy
import gpxpy.gpx
import serial
import os
import csv
# Display initialization
serial_interface = i2c(port=1, address=0x3C)
display = ssd1306(serial_interface)
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

    return x / 16384.0, y / 16384.0, z / 16384.0


initialize_icm()

# GPS initialization
uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=0.5)

# GPIO setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
SCREEN_PIN = 16
ACTIVITY_PIN = 6
GPIO.setup(SCREEN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ACTIVITY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


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

    def nmea_to_decimal(self, raw_val, direction):
        """Convert NMEA latitude/longitude to decimal degrees."""
        if not raw_val or not direction:
            return None
        try:
            raw_val = float(raw_val)
            degrees = int(raw_val // 100)
            minutes = raw_val - degrees * 100
            decimal = degrees + minutes / 60
            if direction in ['S', 'W']:
                decimal *= -1
            return decimal
        except ValueError:
            return None

    def update_from_nmea(self, sentence):
        try:
            msg = pynmea2.parse(sentence)
            if isinstance(msg, pynmea2.types.talker.GGA):
                self.latitude = self.nmea_to_decimal(msg.lat, msg.lat_dir)
                self.longitude = self.nmea_to_decimal(msg.lon, msg.lon_dir)
                self.altitude = float(msg.altitude) if msg.altitude else 0.0
                self.has_fix = int(msg.gps_qual) > 0 if msg.gps_qual else False
                self.satellites = int(msg.num_sats) if msg.num_sats else 0

            elif isinstance(msg, pynmea2.types.talker.RMC):
                if msg.lat and msg.lon:
                    self.latitude = self.nmea_to_decimal(msg.lat, msg.lat_dir)
                    self.longitude = self.nmea_to_decimal(msg.lon, msg.lon_dir)
                    if msg.spd_over_grnd:
                        self.speed_knots = float(msg.spd_over_grnd)
                        self.speed_kmh = self.speed_knots * 1.852
                        if isinstance(msg.timestamp, datetime.time):
                            self.timestamp = datetime.datetime.combine(datetime.datetime.today(), msg.timestamp)
        except (pynmea2.ParseError, AttributeError, ValueError):
            pass


gps_data = GPSData()


class ActivityTracker:
    def __init__(self):
        self.active = False
        self.start_time = None
        self.gpx = None
        self.gpx_segment = None
        self.csv_file = None
        self.csv_writer = None
        self.display_avg_speed = 0.0
        self.display_max_speed = 0.0
        self.point_count = 0
        self.last_avg_update = 0

    def start(self):
        self.active = True
        self.start_time = datetime.datetime.now()

        # Initialize GPX
        self.gpx = gpxpy.gpx.GPX()
        self.gpx.creator = "RPi Activity Tracker"
        gpx_track = gpxpy.gpx.GPXTrack()
        self.gpx.tracks.append(gpx_track)
        self.gpx_segment = gpxpy.gpx.GPXTrackSegment()
        gpx_track.segments.append(self.gpx_segment)

        # Initialize CSV
        os.makedirs("activity_data", exist_ok=True)
        csv_filename = f"activity_data/speeds_{self.start_time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(csv_filename, 'w', newline='', buffering=8192)
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'speed_kmh'])

        # Reset stats
        self.display_avg_speed = 0.0
        self.display_max_speed = 0.0
        self.point_count = 0
        self.last_avg_update = time.time()

        print(f"Activity started. CSV: {csv_filename}")

    def stop(self):
        if not self.active:
            return None

        self.active = False

        # Calculate final statistics
        csv_filename = self.csv_file.name
        self.csv_file.close()
        final_avg, final_max = self._calculate_stats_from_csv(csv_filename)

        # Save GPX
        os.makedirs("gpx_logs", exist_ok=True)
        gpx_filename = f"gpx_logs/activity_{self.start_time.strftime('%Y%m%d_%H%M%S')}.gpx"
        with open(gpx_filename, "w") as f:
            f.write(self.gpx.to_xml())

        print(f"Activity stopped. Saved to {gpx_filename}")
        print(f"Points logged: {self.point_count}")
        print(f"Average speed: {final_avg:.2f} km/h")
        print(f"Max speed: {final_max:.2f} km/h")

        return gpx_filename

    def _calculate_stats_from_csv(self, csv_filename):
        try:
            total_speed = 0.0
            count = 0
            max_speed = 0.0

            with open(csv_filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    speed = float(row['speed_kmh'])
                    if speed > 0:
                        total_speed += speed
                        count += 1
                        max_speed = max(max_speed, speed)

            avg_speed = total_speed / count if count > 0 else 0.0
            return avg_speed, max_speed
        except Exception as e:
            print(f"Error calculating stats: {e}")
            return 0.0, 0.0

    def add_point(self, lat, lon, elev, speed_kmh=None):
        if not self.active:
            return

        point = gpxpy.gpx.GPXTrackPoint(
            latitude=lat,
            longitude=lon,
            elevation=elev,
            time=datetime.datetime.utcnow()
        )
        self.gpx_segment.points.append(point)
        self.point_count += 1

        if speed_kmh and speed_kmh > 0:
            self.csv_writer.writerow([datetime.datetime.now().isoformat(), speed_kmh])
            if speed_kmh > self.display_max_speed:
                self.display_max_speed = speed_kmh

    def update_display_stats(self):
        if not self.active:
            return

        now = time.time()
        if now - self.last_avg_update < 10.0:
            return

        self.last_avg_update = now
        self.csv_file.flush()

        try:
            with open(self.csv_file.name, 'r') as f:
                reader = csv.DictReader(f)
                speeds = [float(row['speed_kmh']) for row in reader if float(row['speed_kmh']) > 0]
                recent_speeds = speeds[-30:] if len(speeds) > 30 else speeds
                if recent_speeds:
                    self.display_avg_speed = sum(recent_speeds) / len(recent_speeds)
        except Exception:
            pass

    def get_duration(self):
        if not self.active or not self.start_time:
            return "00:00:00"
        delta = datetime.datetime.now() - self.start_time
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
    now = datetime.datetime.now()
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
        draw.text((0, 16), "Press to START", font=font, fill=255)
    else:
        draw.text((0, 0), f"Time: {activity_tracker.get_duration()}", font=font, fill=255)
        draw.text((0, 16), f"Avg: {activity_tracker.display_avg_speed:.1f}km/h", font=font, fill=255)
        draw.text((0, 32), f"Max: {activity_tracker.display_max_speed:.1f}km/h", font=font, fill=255)
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
last_display_update = 0
last_accel_read = 0
last_button_check = 0

# Performance tuning intervals
LOG_INTERVAL = 3.0
DISPLAY_INTERVAL = 1.0
ACCEL_INTERVAL = 1.0
BUTTON_INTERVAL = 0.2
GPS_READ_INTERVAL = 0.5

ax = ay = az = 0.0

try:
    last_gps_read = 0

    while True:
        now = time.time()

        # Read GPS data
        if now - last_gps_read >= GPS_READ_INTERVAL:
            try:

                if uart.in_waiting > 0:
                    try:
                        line = uart.readline().decode('ascii', errors='replace').strip()
                        if line:
                            gps_data.update_from_nmea(line)
                    except (UnicodeDecodeError, serial.SerialException):
                        pass
                    last_gps_read = now
            except OSError:
                uart.reset_input_buffer()
                continue

        # Check buttons
        if now - last_button_check >= BUTTON_INTERVAL:
            if GPIO.input(SCREEN_PIN) == GPIO.LOW:
                screen_index = (screen_index + 1) % screen_count
                time.sleep(0.3)
                while GPIO.input(SCREEN_PIN) == GPIO.LOW:
                    time.sleep(0.05)

            if GPIO.input(ACTIVITY_PIN) == GPIO.LOW:
                time.sleep(0.3)
                while GPIO.input(ACTIVITY_PIN) == GPIO.LOW:
                    time.sleep(0.05)

                if activity_tracker.active:
                    activity_tracker.stop()
                else:
                    if gps_data.has_fix:
                        activity_tracker.start()
                    else: 
                        last_button_check = now

        # Log GPS points
        if activity_tracker.active and gps_data.has_fix and now - last_log_time >= LOG_INTERVAL:
            if gps_data.latitude is not None and gps_data.longitude is not None:
                activity_tracker.add_point(
                    gps_data.latitude,
                    gps_data.longitude,
                    gps_data.altitude if gps_data.altitude else 0,
                    gps_data.speed_kmh
                )
            last_log_time = now

        if activity_tracker.active:
            activity_tracker.update_display_stats()

        # Read accelerometer
        if now - last_accel_read >= ACCEL_INTERVAL:
            try:
                ax, ay, az = read_accel()
            except Exception as e:
                last_accel_read = now

        # Display update
        if now - last_display_update >= DISPLAY_INTERVAL:
            try:
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
            except Exception as e:
                last_display_update = now

        time.sleep(0.05)

except KeyboardInterrupt:
    if activity_tracker.active:
        activity_tracker.stop()
    GPIO.cleanup()
    uart.close()
    bus_bme.close()
    bus_icm.close()
