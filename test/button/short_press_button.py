import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)

screen_index = 0
num_screen = 3

while True:
    if GPIO.input(21) == GPIO.LOW:
        screen_index = (screen_index + 1) % num_screen
        print(f"Screen index: {screen_index}")
        time.sleep(0.3)  # debounce

        # Wait for button release before next loop
        while GPIO.input(21) == GPIO.LOW:
            time.sleep(0.01)

