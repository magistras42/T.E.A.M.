import cv2
import math
import numpy as np
import video_processing as vp
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt

#citations

#User raja_961, “Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV”. Instructables.
#URL: https://www.instructables.com/Autonomous-Lane-Keeping-Car-U sing-Raspberry-Pi-and/

#Team Houston Dynamics, "We Have Cybertruck at Home". 
#URL: https://www.hackster.io/houston-dynamics-spencer-darwall-noah-elzner-agha-sukerim-anthony-zheng/we-have-cybertruck-at-home-0e429f

#For help with plotting code only, we consulted:
#Team M.E.G.G., "The Magnificent M.E.G.G. Car". 
#URL: https://www.hackster.io/m-e-g-g/the-magnificent-m-e-g-g-car-28ec89

MAX_TICK_COUNT = 1 << 12

# PD variables
PD_KP = 0.1 ##proportional gain
PD_KD = PD_KP * 0.01 #derivative gain

p_vals = [] # proportional
d_vals = [] # Derivative
err_vals_1 = [] # error
speed_vals = [] # speed values
steer_vals = [] # steering values
speed_err = []

### SPEED CONSTANTS
HALT_SPEED = 7.5
IDEAL_SPEED = 7.8
THROTTLE_MAX = 8.0
THROTTLE_MIN = 7.5
THROTTLE_TICKS = 3

### STEERING CONSTANTS
STEER_CENTER = 7.5
STEER_DIFF = 1.5

### VIDEO PROCESSING CONSTANTS
BUFSIZE = 10
LARGE_NUMBER = 100 # number for red light detection

### ENCODER PROCESSING CONSTANTS
ENCODER_PARAM = f"/sys/module/speed_driver/parameters/elapsedTime"
ENCODER_TARGET = 3000
ENCODER_MIN = 10000
ENCODER_MAX = 40000
SPEED_DELTA = 0.004
SLOW_FACTOR = 0.5

# Steering Motor Pins
steering_enable = 19
# Throttle Motors Pins
throttle_enable = 18
cur_throttle = IDEAL_SPEED

# Initialize steering buffer.
steer_buf = [STEER_CENTER] * BUFSIZE
steer_idx = 0
steer_sum = sum(steer_buf)
start_time = time.time()

"""
Get speed time differential from ENCODER_PARAM.
"""
def read_encoder() -> int:
    # Open file, read parameter.
    with open(ENCODER_PARAM) as encoder:
        res = encoder.read().strip()
        # If early in start time, just return ideal encoder target.
        if time.time() - start_time < 2:
            return ENCODER_TARGET
        print(f"ENCODER VALUE = {res}")
        return int(res)
"""
Calculate encoder error for debugging purposes.
"""
def get_encoder_error() -> int:
    retval = read_encoder() - ENCODER_TARGET
    return retval

"""
Add a given sample to processing buffer.
"""    
def add_sample_to_buf(sample: int):
    global steer_sum
    global steer_idx
    # Update total steering sum of sliding window
    steer_sum -= steer_buf[steer_idx]
    steer_sum += sample
    # Update steering buffer sliding window idx
    steer_buf[steer_idx] = sample
    steer_idx = (steer_idx + 1) % BUFSIZE

"""
Calculate the average of the sliding window.
"""    
def get_average_sample():
    global steer_sum
    return steer_sum / BUFSIZE

"""
Create PD plot.
"""
def plot_pd(p_vals, d_vals, error, show_img=False):
    # Plot the proportional, derivative and error
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(p_vals))
    ax1.plot(t_ax, p_vals, '-', label="P values")
    ax1.plot(t_ax, d_vals, '-', label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    # Format axes.
    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    # Format overall graph.
    plt.title("PD Values over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("pd_plot.png")

    # Display image.
    if show_img:
        plt.show()
    plt.clf()

"""
Create PWM plot.
"""
def plot_pwm(speed_pwms, turn_pwms, error, speed_error, show_img=False):
    # Plot the speed steering and the error
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(speed_pwms))
    ax1.plot(t_ax, speed_pwms, '-', label="Speed")
    ax1.plot(t_ax, turn_pwms, '-', label="Steering")
    ax2 = ax1.twinx()
    
    # Plot steering and encoder errors.
    ax2.plot(t_ax, error / max(np.max(error), abs(np.min(error))), '--r', label="Steering Error")
    ax2.plot(t_ax, speed_error / max(np.max(speed_error), abs(np.min(speed_error))), '--b', label="Encoder Error")

    # Format axes.
    ax1.set_xlabel("Frames")
    ax1.set_ylabel("Speed and Steer Duty Cycle")
    ax2.set_ylabel("Error Value")

    # Format overall graph.
    plt.title("Speed and Steer Duty Cycle, and error v.s. time")
    fig.legend()
    plt.savefig("voltage_plot.png")

    # Disply image, if desired.
    if show_img:
    	plt.show()

"""
Initialize the car.
"""    
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

"""
Main driver code
"""
def main():
    # Initialize car
    last_time = 0
    last_error = 0
    throttle, steering = init_car()

    # Initialize video processing
    video = vp.init_video()
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 176)
    
    # After processing set up, make sure at rest before beginning.
    throttle.ChangeDutyCycle(HALT_SPEED)
    time.sleep(3)
    turn_amt = 7.5

    try:
        # Start moving on track. 
        curr_speed = IDEAL_SPEED
        curr_steer = 7.5

        throttle.ChangeDutyCycle(curr_speed)
        steering.ChangeDutyCycle(curr_steer)

        # Initialize track details to watch for
        time_since_red_light = LARGE_NUMBER
        red_light_count = 0
        curr_tick = 0
        sp_err = 0

        # Run essentially forever, but not using a while true for
        # tear down problems.
        while curr_tick < MAX_TICK_COUNT:
            key = cv2.waitKey(1)
            time_since_red_light += 0.1

            # Update speed control every THROTTLE_TICKS loops.
            if (curr_tick % THROTTLE_TICKS) == 0:
                # Speed control 
                enc_val = read_encoder()
                sp_err = 0
                if enc_val >= ENCODER_MAX: # Increase speed
                    sp_err = 1
                    # Speed up if going too slow
                    if curr_speed + SPEED_DELTA * THROTTLE_TICKS <= THROTTLE_MAX:
                        curr_speed += SPEED_DELTA * THROTTLE_TICKS
                        throttle.ChangeDutyCycle(curr_speed)
                        print("SLOW")
                # Speed up if going too fast.
                elif enc_val <= ENCODER_MIN:
                    sp_err = -1
                    # Only correct speed if going too slow in this case.
                    if curr_speed - (SPEED_DELTA * THROTTLE_TICKS) >= THROTTLE_MIN:
                        curr_speed -= (SLOW_FACTOR * SPEED_DELTA * THROTTLE_TICKS)
                        throttle.ChangeDutyCycle(curr_speed)
                        print("FAST")

            # from 0 to 180
            ret, frame = video.read()
            read_angle = vp.calculate_steering_angle(frame, turn_amt)
            
            # from -90 to 90
            our_angle = read_angle - 90

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
            speed_vals.append(curr_speed)
            speed_err.append(sp_err)

            last_error = error 

            # PD Control!
            turn_amt = STEER_CENTER + proportional + derivative
            turn_amt = max(STEER_CENTER - STEER_DIFF, turn_amt)
            turn_amt = min(STEER_CENTER + STEER_DIFF, turn_amt)
            steer_vals.append(turn_amt)

            # Update steering.
            add_sample_to_buf(turn_amt)
            avg = get_average_sample()
            steering.ChangeDutyCycle(avg)

            # Detect Redlight
            if(time_since_red_light > 50 and (curr_tick % 5) == 0 and vp.is_red(frame)):
                print("red light detect")

                # First stop paper encountered
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

        # Release video processing resources.
        video.release()
        cv2.destroyAllWindows()
        
        # Release throttle and steering resources.
        throttle.stop()
        steering.stop()
        GPIO.cleanup([steering_enable, throttle_enable])

        # Create plots
        plot_pd(p_vals, d_vals, err_vals_1, show_img=True)
        plot_pwm(speed_vals, steer_vals, err_vals_1, speed_err, show_img=True)
        
    except KeyboardInterrupt or Exception:
        # Handling for end to program
        print("Stopping program...")
        
        # Free resources
        video.release()
        cv2.destroyAllWindows()
        throttle.stop()
        steering.stop()
        GPIO.cleanup([steering_enable, throttle_enable])

        # Create plots
        plot_pd(p_vals, d_vals, err_vals_1, show_img=True)
        plot_pwm(speed_vals, steer_vals, err_vals_1, speed_err, show_img=True)

if __name__ == "__main__":
    main()


