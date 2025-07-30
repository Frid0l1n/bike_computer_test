import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up resistor enabled

while True:
    if GPIO.input(17) == GPIO.LOW:  # Button pressed
        print("Button was pushed!")
        time.sleep(0.3)  # debounce delay

