import cv2
import math
import numpy as np
import video_processing as vp
import RPi.GPIO as GPIO
import time

# Constants
MAX_TICK_COUNT = 1 << 12
PD_KP = 0.09
PD_KD = PD_KP * 0.5
SPD_KP = 0.09
SPD_KD = SPD_KP * 0.5

HALT_SPEED = 7.5
THROTTLE_MAX = 8.0
ENCODER_SENSITIVITY = 7e-5

STEER_CENTER = 7.5
STEER_DIFF = 1.5
BUFSIZE = 10
ENCODER_PARAM = "/sys/module/speed_driver/parameters/elapsedTime"
ENCODER_TARGET = 6000000

# GPIO pins
steering_enable = 19
throttle_enable = 18

# Global state
p_vals, d_vals, err_vals_1 = [], [], []
speed_err_vals, steer_vals = [], []
steer_buf = [STEER_CENTER] * BUFSIZE
steer_sum = sum(steer_buf)
steer_idx = 0
cur_throttle = 7.8
last_speed_error = 0
start_time = time.time()

# Setup
def init_car():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(throttle_enable, GPIO.OUT)
    GPIO.setup(steering_enable, GPIO.OUT)

    throttle = GPIO.PWM(throttle_enable, 50)
    steering = GPIO.PWM(steering_enable, 50)

    throttle.start(HALT_SPEED)
    steering.start(STEER_CENTER)
    return throttle, steering

def read_encoder() -> int:
    with open(ENCODER_PARAM) as encoder:
        res = encoder.read().strip()
        if time.time() - start_time < 2:
            return ENCODER_TARGET
        print(f"ENCODER VALUE = {res}")
        return int(res)

# Throttle control
def pd_throttle_control(current_time):
    global last_speed_error, cur_throttle

    enc_val = read_encoder()
    error = ENCODER_TARGET - enc_val
    speed_err_vals.append(error)

    now = time.time()
    dt = now - current_time if current_time > 0 else 0.04

    proportional = SPD_KP * error
    derivative = SPD_KD * (error - last_speed_error) / dt
    last_speed_error = error

    throttle_change = proportional + derivative
    cur_throttle = min(max(cur_throttle + throttle_change, HALT_SPEED), THROTTLE_MAX)

    print(f"[Throttle PD] Error: {error}, Throttle: {cur_throttle:.3f}")
    return cur_throttle

# Steering control
def add_sample_to_buf(sample: float):
    global steer_sum, steer_idx
    steer_sum -= steer_buf[steer_idx]
    steer_sum += sample
    steer_buf[steer_idx] = sample
    steer_idx = (steer_idx + 1) % BUFSIZE

def get_average_sample() -> float:
    return steer_sum / BUFSIZE

def update_steering(steering, angle, last_error, dt, curr_steer):
    error = angle
    proportional = PD_KP * error
    derivative = PD_KD * (error - last_error) / dt

    turn_amt = STEER_CENTER + proportional + derivative
    turn_amt = max(STEER_CENTER - STEER_DIFF, min(STEER_CENTER + STEER_DIFF, turn_amt))

    add_sample_to_buf(turn_amt)
    avg = get_average_sample()

    if abs(avg - curr_steer) > 0.05:
        steering.ChangeDutyCycle(avg)
        curr_steer = avg

    # Debug and logging
    p_vals.append(proportional)
    d_vals.append(derivative)
    err_vals_1.append(error)
    steer_vals.append(turn_amt)

    return error, curr_steer

# Main control loop
def main():
    throttle, steering = init_car()
    video = vp.init_video()
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    throttle.ChangeDutyCycle(HALT_SPEED)
    time.sleep(3)

    curr_steer = STEER_CENTER
    curr_tick = 0
    last_time = 0
    last_error = 0

    try:
        throttle.ChangeDutyCycle(cur_throttle)
        steering.ChangeDutyCycle(curr_steer)

        while curr_tick < MAX_TICK_COUNT:
            key = cv2.waitKey(1)
            read_angle = vp.calculate_steering_angle(video)
            our_angle = read_angle - 90

            now = time.time()
            dt = now - last_time
            last_time = now

            last_error, curr_steer = update_steering(steering, our_angle, last_error, dt, curr_steer)

            if (curr_tick % 2) == 0:
                new_speed = pd_throttle_control(last_time)
                throttle.ChangeDutyCycle(new_speed)

            curr_tick += 1

        video.release()
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        print("Stopping program...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        throttle.stop()
        steering.stop()
        GPIO.cleanup([steering_enable, throttle_enable])

if __name__ == "__main__":
    main()

