import time
import RPi.GPIO as GPIO

# Set pin numbering convention
GPIO.setmode(GPIO.BCM)

# GPIO pin to test
test_point = 21

# Set the pin as an output
GPIO.setup(test_point, GPIO.OUT)

try:
    while True:
        s = input("Turn on (1) or off (0), q to quit: ").strip().lower()

        if s == "q":
            break

        elif s == "1":
            GPIO.output(test_point, GPIO.HIGH)
            print("Solenoid ON")
            time.sleep(1)

        elif s == "0":
            GPIO.output(test_point, GPIO.LOW)
            print("Solenoid OFF")
            time.sleep(1)

        else:
            print("Invalid input. Type 1, 0, or q.")

except KeyboardInterrupt:
    pass

finally:
    GPIO.output(test_point, GPIO.LOW)  # safety: turn off before cleanup
    GPIO.cleanup()
    print("GPIO cleaned up.")