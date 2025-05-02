import cv2
import math
import numpy as np
import video_processing as vp
import RPi.GPIO as GPIO
import time
import matplotlib as plt

MAX_TICK_COUNT = 1 << 12

# PD variables
# PD_KP = 0.07 ##proportional gain
# PD_KD = PD_KP * 0.03 #derivative gain
PD_KP = 0.09 ##proportional gain
PD_KD = PD_KP * 0.35 #derivative gain

p_vals = [] # proportional
d_vals = [] # Derivative
err_vals_1 = [] # error
err_vals_2 = [] # error
speed_vals = [] # speed values
steer_vals = [] # steering values

HALT_SPEED = 7.5
IDEAL_SPEED = 7.8
THROTTLE_MAX = 7.9
THROTTLE_MIN = 7.7
ENCODER_SENSITIVITY = 7e-5


STEER_CENTER = 7.5
STEER_DIFF = 1.5

BUFSIZE = 10
LARGE_NUMBER = 100 # number for red light detection

ENCODER_PARAM = f"/sys/module/speed_driver/parameters/elapsedTime"
ENCODER_TARGET = 150000
ENCODER_MIN = 8000
ENCODER_MAX = 14000
SPEED_DELTA = 0.015

# Steering Motor Pins
steering_enable = 19
# Throttle Motors Pins
throttle_enable = 18
cur_throttle = IDEAL_SPEED

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

def plot_pd(p_vals, d_vals, error, show_img=False):
    # Plot the proportional, derivative and error
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(p_vals))
    ax1.plot(t_ax, p_vals, '-', label="P values")
    ax1.plot(t_ax, d_vals, '-', label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("PD Values over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("pd_plot.png")

    if show_img:
    	plt.show()
    plt.clf()

def plot_pwm(speed_pwms, turn_pwms, error, show_img=False):
    # Plot the speed steering and the error
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(speed_pwms))
    ax1.plot(t_ax, speed_pwms, '-', label="Speed")
    ax1.plot(t_ax, turn_pwms, '-', label="Steering")
    ax2 = ax1.twinx()
    
    
    ax1.plot(t_ax, error / np.max(error), '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("Speed and Steer Duty Cycle")
    ax2.set_ylabel("Percent Error Value")

    plt.title("Speed and Steer Duty Cycle, and error v.s. time")
    fig.legend()
    plt.savefig("voltage_plot.png")

    if show_img:
    	plt.show()
    plt.clf()

    
def init_car():
    GPIO.setwarnings(False)

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
        curr_speed = IDEAL_SPEED
        curr_steer = 7.5

        throttle.ChangeDutyCycle(curr_speed)
        steering.ChangeDutyCycle(curr_steer)

        time_since_red_light = LARGE_NUMBER
        red_light_count = 0
        curr_tick = 0

        while curr_tick < MAX_TICK_COUNT:
            key = cv2.waitKey(1)
            time_since_red_light += 0.1

            # time.sleep(0.04)
            # thr = update_throttle_value()
            # throttle.ChangeDutyCycle(thr)

            if (curr_tick % 5) == 0:
                # Speed control 
                enc_val = read_encoder()
                if enc_val >= ENCODER_MAX: # Increase speed
                    if curr_speed + SPEED_DELTA <= THROTTLE_MAX:
                        curr_speed += SPEED_DELTA
                        throttle.ChangeDutyCycle(curr_speed)
                elif enc_val <= ENCODER_MIN:
                    if curr_speed - SPEED_DELTA >= THROTTLE_MIN:
                        curr_speed -= SPEED_DELTA
                        throttle.ChangeDutyCycle(curr_speed)
                print("Throttle: ", curr_speed)

            # from 0 to 180
            ret, frame = video.read()
            read_angle = vp.calculate_steering_angle(frame)
            
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
            print("Steering ", avg)
            # diff_steer = avg - curr_steer
            # if diff_steer > 0.05 or diff_steer < -0.05:
            steering.ChangeDutyCycle(avg)
                # curr_steer = avg

            
            # print(f"average_sample={avg}, {our_angle=}, {turn_amt=}")

            # print(turn_amt)

            # Detect Redlight
            if(time_since_red_light > 50 and (curr_tick % 5) == 0 and vp.is_red(frame)):
                print("red light detect")
            
                if(red_light_count == 0):               
            	    #for first sign, stop then restart
                    throttle.ChangeDutyCycle(HALT_SPEED)
                    time.sleep(3)
                    throttle.ChangeDutyCycle(IDEAL_SPEED)
                    time_since_red_light = 0
                    red_light_count += 1
                else:
            	    #for second time, stop then turn car off
                    throttle.ChangeDutyCycle(HALT_SPEED)
                    break
            

            curr_tick += 1

        video.release()
        cv2.destroyAllWindows()
        plot_pd(p_vals, d_vals, error, show_img=True)
        plot_pwm(speed_vals, steer_vals, error, show_img=True)
        
    except KeyboardInterrupt or Exception:
        # Handling for end to program
        print("Stopping program...")
        
        throttle.stop()
        steering.stop()

        GPIO.cleanup([steering_enable, throttle_enable])

if __name__ == "__main__":
    main()


