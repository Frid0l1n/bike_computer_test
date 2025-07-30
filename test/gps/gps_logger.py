import time
import serial
import csv
import adafruit_gps

# Setup serial connection to GPS
uart = serial.Serial("/dev/serial0", baudrate=9600, timeout=30)

# Initialize GPS module
gps = adafruit_gps.GPS(uart, debug=False)

# Configure GPS output and update rate
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')  # Output RMC and GGA sentences
gps.send_command(b'PMTK220,1000')  # 1 Hz update rate

# Create CSV file and write header
with open('gps_data.csv', mode='w', newline='') as csvfile:
    fieldnames = ['timestamp', 'latitude', 'longitude']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    last_print = time.monotonic()
    while True:
        gps.update()
        current = time.monotonic()

        if current - last_print >= 1.0:
            last_print = current
            if not gps.has_fix:
                print('Waiting for fix...')
                continue

            # Print GPS data
            print()
            print(f'Latitude: {gps.latitude:.6f} degrees')
            print(f'Longitude: {gps.longitude:.6f} degrees')

            # Write to CSV
            writer.writerow({
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'latitude': gps.latitude,
                'longitude': gps.longitude
            })
            csvfile.flush()  # Make sure it's written to disk in real time

