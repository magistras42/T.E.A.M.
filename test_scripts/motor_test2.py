import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)


# Steering Motor Pins
steering_enable = 19
in1 = 17 # Physical Pin 11
in2 = 27 # Physical Pin 13

#Throttle Motors Pins
throttle_enable = 18
in3 = 23 # Physical Pin 16
in4 = 24 # Physical Pin 18

GPIO.setmode(GPIO.BCM) # Use GPIO numbering instead of physical numbering
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(throttle_enable, GPIO.OUT)
GPIO.setup(steering_enable, GPIO.OUT)

# Steering Motor Control
GPIO.output(in1, GPIO.HIGH)
GPIO.output(in2, GPIO.LOW)
steering = GPIO.PWM(steering_enable, 50) # set the switching frequency to 50 Hz
steering.stop()

# Throttle Motors Control
GPIO.output(in3, GPIO.HIGH)
GPIO.output(in4, GPIO.LOW)
throttle = GPIO.PWM(throttle_enable, 50) # set the switching frequency to 50 Hz
throttle.stop()

time.sleep(1)

throttle.start(7.5) # starts the motor at 7.5% PWM signal-> (0.075 * battery Voltage) - driver's loss
steering.start(7.5) # starts the motor at 7.5% PWM signal-> (0.075 * Battery Voltage) - driver's loss

for step in range(1):
    percent = 2.5 + step * 0.5
    steering.ChangeDutyCycle(percent)
    print(percent)
    time.sleep(0.5)

time.sleep(2)

throttle.stop()
steering.stop()

GPIO.cleanup([steering_enable, throttle_enable])
