import cv2
import math
import numpy as np
import video_processing as vp
import time

# PD variables
PD_KP = 0.09 ##proportional gain
PD_KD = PD_KP * 0.5 #derivative gain

p_vals = [] # proportional
d_vals = [] # Derivative
err_vals_1 = [] # error
err_vals_2 = [] # error
speed_vals = [] # speed values
steer_vals = [] # steering values


def main():
    lastTime = 0
    last_error = 0

    video = vp.init_video()

    while True:
        # from 0 to 180
        read_angle = vp.calculate_steering_angle(video)
        
        # from -90 to 90
        our_angle = read_angle - 90

        # dt calculation
        now = time.time()
        dt = now - lastTime

        # PD Code
        error = our_angle
        base_turn = 7.5
        proportional = PD_KP * error
        derivative = PD_KD * (error - last_error) / dt

        # Append values
        p_vals.append(proportional)
        d_vals.append(derivative)
        err_vals_1.append(error)
        # speed_vals.append(curr_speed)

        last_error = error 

        # PD Control!
        turn_amt = base_turn + proportional + derivative
        steer_vals.append(turn_amt)

        print(turn_amt)
        # FIXME: Turn this amount

        # Detect Redlight



        time.sleep(0.1)

        key = cv2.waitKey(1)
        if key == 27:
            break

    video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


