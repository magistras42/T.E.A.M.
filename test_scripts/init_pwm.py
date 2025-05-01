import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)


# Steering Motor Pins
steering_enable = 19

#Throttle Motors Pins
throttle_enable = 18

GPIO.setmode(GPIO.BCM) # Use GPIO numbering instead of physical numbering
GPIO.setup(throttle_enable, GPIO.OUT)
GPIO.setup(steering_enable, GPIO.OUT)

# Steering Motor Control
steering = GPIO.PWM(steering_enable, 50) # set the switching frequency to 50 Hz

# Throttle Motors Control
throttle = GPIO.PWM(throttle_enable, 50) # set the switching frequency to 50 Hz

throttle.start(7.5) # starts the motor at 5% PWM signal-> (0.05 * battery Voltage) - driver's loss
steering.start(7.5) # starts the motor at 7.5% PWM signal-> (0.075 * Battery Voltage) - driver's loss

time.sleep(1)

throttle.changeDutyCycle(10)
time.sleep(2)
throttle.changeDutyCycle(5)
time.sleep(2)
throttle.changeDutyCycle(7.5)
time.sleep(2)

print("you have been stopped fool")

throttle.stop()
steering.stop()

GPIO.cleanup([steering_enable, throttle_enable])
