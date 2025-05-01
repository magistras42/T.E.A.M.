import cv2
import math
import numpy as np
import video_processing as vp
import RPi.GPIO as GPIO
import time

# PD variables
PD_KP = 0.04 ##proportional gain
PD_KD = PD_KP * 0.5 #derivative gain

p_vals = [] # proportional
d_vals = [] # Derivative
err_vals_1 = [] # error
err_vals_2 = [] # error
speed_vals = [] # speed values
steer_vals = [] # steering values

HALT_SPEED = 7.5
IDEAL_SPEED = 7.8
STEER_CENTER = 7.5
STEER_DIFF = 1.5


def init_car():
    GPIO.setwarnings(False)

    # Steering Motor Pins
    steering_enable = 19
    # Throttle Motors Pins
    throttle_enable = 18

    GPIO.setmode(GPIO.BCM) # Use GPIO numbering instead of physical numbering
    GPIO.setup(throttle_enable, GPIO.OUT)
    GPIO.setup(steering_enable, GPIO.OUT)

    # Steering Motor Control
    steering = GPIO.PWM(steering_enable, 50) # set the switching frequency to 50 Hz

    # Throttle Motors Control
    throttle = GPIO.PWM(throttle_enable, 50) # set the switching frequency to 50 Hz

    throttle.start(HALT_SPEED) # starts the motor at 7.5% PWM signal-> (0.075 * battery Voltage) - driver's loss
    steering.start(STEER_CENTER) # starts the motor at 7.5% PWM signal-> (0.075 * Battery Voltage) - driver's loss

    return throttle, steering

def adjust_steering(control_val, steering):
    if control_val < -45.0: control_val = -45.0
    elif control_val > 45.0: control_val = 45.0
    steer_vals.append(control_val)
    our_diff = (control_val) / 45.0
    val = STEER_CENTER + (STEER_DIFF * our_diff)
    # print(val, control_val)
    steering.ChangeDutyCycle(val)

def main():
    last_time = 0
    last_error = 0

    video = vp.init_video()
    time.sleep(1.5)

    throttle, steering = init_car()

    try: 
        throttle.ChangeDutyCycle(7.8)

        while True:
            time.sleep(0.25)
            throttle.ChangeDutyCycle(7.6)

            # from 0 to 180
            read_angle = vp.calculate_steering_angle(video)
            
            # from -90 to 90
            our_angle = read_angle - 90

            # print(read_angle, our_angle)

            # dt calculation
            now = time.time()
            dt = now - last_time
            last_time = now

            # PD Code
            error = our_angle
            proportional = PD_KP * error
            derivative = PD_KD * (error - last_error) / dt

            # Append values
            p_vals.append(proportional)
            d_vals.append(derivative)
            err_vals_1.append(error)
            # speed_vals.append(curr_speed)

            last_error = error 

            # PD Control!
            turn_amt = STEER_CENTER + proportional + derivative
            turn_amt = max(STEER_CENTER - STEER_DIFF, turn_amt)
            turn_amt = min(STEER_CENTER + STEER_DIFF, turn_amt)
            steer_vals.append(turn_amt)

            print(our_angle, turn_amt)

            # STEER!
            steering.ChangeDutyCycle(turn_amt)

            # print(turn_amt)

            # Detect Redlight

            key = cv2.waitKey(1)
            if key == 27:
                break

        video.release()
        cv2.destroyAllWindows()
        
    except KeyboardInterrupt or Exception:
        # Handling for end to program
        print("Stopping program...")
        
        throttle.stop()
        steering.stop()

        GPIO.cleanup([steering_enable, throttle_enable])

        throttleValues.write(str(0) + '\n')

if __name__ == "__main__":
    main()


