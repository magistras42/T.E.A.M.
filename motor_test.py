import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)


# Steering Motor Pins
steering_enable = 22 # Physical Pin 15
in1 = 17 # Physical Pin 11
in2 = 27 # Physical Pin 13

#Throttle Motors Pins
throttle_enable = 25 # Physical Pin 22
in3 = 23 # Physical Pin 16
in4 = 24 # Physical Pin 18

GPIO.setmode(GPIO.BCM) # Use GPIO numbering instead of physical numbering
GPIO.setup(in1, GPIO.out)
GPIO.setup(in2, GPIO.out)
GPIO.setup(in3, GPIO.out)
GPIO.setup(in4, GPIO.out)
GPIO.setup(throttle_enable, GPIO.out)
GPIO.setup(steering_enable, GPIO.out)

# Steering Motor Control
GPIO.output(in1, GPIO.HIGH)
GPIO.output(in2, GPIO.LOW)
steering = GPIO.PWM(steering_enable, 1000) # set the switching frequency to 1000 Hz
steering.stop()

# Throttle Motors Control
GPIO.output(in3, GPIO.HIGH)
GPIO.output(in4, GPIO.LOW)
throttle = GPIO.PWM(throttle_enable, 1000) # set the switching frequency to 1000 Hz
throttle.stop()

time.sleep(1)

throttle.start(25) # starts the motor at 25% PWM signal-> (0.25 * battery Voltage) - driver's loss
steering.start(100) # starts the motor at 100% PWM signal-> (1 * Battery Voltage) - driver's loss

time.sleep(3)

throttle.stop()
steering.stop()
