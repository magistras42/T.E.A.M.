import cv2
import math
import numpy as np
import video_processing as vp
import RPi.GPIO as GPIO
import time

MAX_TICK_COUNT = 1 << 12

# PD variables
PD_KP = 0.07 ##proportional gain
PD_KD = PD_KP * 0.03 #derivative gain

p_vals = [] # proportional
d_vals = [] # Derivative
err_vals_1 = [] # error
err_vals_2 = [] # error
speed_vals = [] # speed values
steer_vals = [] # steering values

HALT_SPEED = 7.5
THROTTLE_MAX = 8.0
ENCODER_SENSITIVITY = 7e-5

MIN_ENC_PERIOD = 5000
MAX_ENC_PERIOD = 10000
SPEED_DELTA = 0.001

STEER_CENTER = 7.5
STEER_DIFF = 1.5

BUFSIZE = 10

ENCODER_PARAM = f"/sys/module/speed_driver/parameters/elapsedTime"
ENCODER_TARGET = 2000

# Steering Motor Pins
steering_enable = 19
# Throttle Motors Pins
throttle_enable = 18
cur_throttle = 7.8

steer_buf = [STEER_CENTER] * BUFSIZE
steer_idx = 0
steer_sum = sum(steer_buf)
start_time = time.time()

def read_encoder() -> int:
    with open(ENCODER_PARAM) as encoder:
        res = encoder.read().strip()
        if time.time() - start_time < 2:
            return ENCODER_TARGET
        print(f"ENCODER VALUE = {res}")
        return int(res)

def get_encoder_error() -> int:
    return read_encoder() - ENCODER_TARGET

def update_throttle_value() -> int:
    global cur_throttle
    cur_throttle += get_encoder_error() * ENCODER_SENSITIVITY
    cur_throttle = max(cur_throttle, HALT_SPEED)
    cur_throttle = min(cur_throttle, THROTTLE_MAX)
    print(f"{cur_throttle=}")
    return cur_throttle

def calculate_delta_speed():
    enc_val = read_encoder()
    ret_val = 0
    if enc_val >= MAX_ENC_PERIOD: # Increase speed
        ret_val += SPEED_DELTA
    if enc_val <= MIN_ENC_PERIOD: # Decrease speed
        ret_val -= SPEED_DELTA
    return ret_val
    

def add_sample_to_buf(sample: int):
    global steer_sum
    global steer_idx
    steer_sum -= steer_buf[steer_idx]
    steer_sum += sample
    steer_buf[steer_idx] = sample
    steer_idx = (steer_idx + 1) % BUFSIZE
    
def get_average_sample():
    global steer_sum
    return steer_sum / BUFSIZE
    
def init_car():
    GPIO.setwarnings(False)

    GPIO.setmode(GPIO.BCM) # Use GPIO numbering instead of physical numbering
    GPIO.setup(throttle_enable, GPIO.OUT)
    GPIO.setup(steering_enable, GPIO.OUT)

    # Steering Motor Control
    steering = GPIO.PWM(steering_enable, 50) # set the switching frequency to 50 Hz

    # Throttle Motors Control
    throttle = GPIO.PWM(throttle_enable, 50) # set the switching frequency to 50 Hz

    #throttle.start(HALT_SPEED) # starts the motor at 7.5% PWM signal-> (0.075 * battery Voltage) - driver's loss
    steering.start(STEER_CENTER) # starts the motor at 7.5% PWM signal-> (0.075 * Battery Voltage) - driver's loss

    return throttle, steering

def main():
    last_time = 0
    last_error = 0
    throttle, steering = init_car()

    video = vp.init_video()
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    throttle.ChangeDutyCycle(HALT_SPEED)
    time.sleep(3)

    try: 
        curr_speed = 7.833
        curr_steer = 7.5

        #throttle.ChangeDutyCycle(curr_speed)
        steering.ChangeDutyCycle(curr_steer)


        curr_tick = 0

        while curr_tick < MAX_TICK_COUNT:
            key = cv2.waitKey(1)

            # time.sleep(0.04)
            # thr = update_throttle_value()
            # throttle.ChangeDutyCycle(thr)

            #if (curr_tick % 5) == 0:
                ## Speed control 
                #delta_speed = calculate_delta_speed()
                #if delta_speed != 0:
                    #curr_speed += delta_speed
                    #throttle.ChangeDutyCycle(curr_speed)

            # from 0 to 180
            read_angle = vp.calculate_steering_angle(video)
            
            # from -90 to 90
            our_angle = read_angle - 90

            # print(read_angle, our_angle)

            # dt calculation
            now = time.time()
            dt = now - last_time
            # print(f"Timing: {dt}")
            last_time = now

            # PD Code
            error = our_angle
            proportional = PD_KP * error
            derivative = PD_KD * (error - last_error) / dt
            # print(f"{proportional=} {derivative=}")

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

            # STEER!
            add_sample_to_buf(turn_amt)
            avg = get_average_sample()
            print(avg)
            diff_steer = avg - curr_steer
            if diff_steer > 0.05 or diff_steer < -0.05:
                steering.ChangeDutyCycle(avg)
                curr_steer = avg

            
            # print(f"average_sample={avg}, {our_angle=}, {turn_amt=}")

            # print(turn_amt)

            # Detect Redlight

            curr_tick += 1

        video.release()
        cv2.destroyAllWindows()
        
    except KeyboardInterrupt or Exception:
        # Handling for end to program
        print("Stopping program...")
        
        throttle.stop()
        steering.stop()

        GPIO.cleanup([steering_enable, throttle_enable])

if __name__ == "__main__":
    main()


