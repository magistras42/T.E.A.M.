import cv2
import numpy as np
import math
import time
import Adafruit_BBIO.PWM as PWM


def convert_to_HSV(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV", hsv)
    return hsv


def detect_edges(frame):
    # lower limit of blue color
    lower_blue = np.array([90, 120, 0], dtype="uint8")
    # upper limit of blue color
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    # this mask will filter out everything but blue
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    #cv2.imshow("edges", edges)
    return edges


def detect_red(frame):
    # low_red = np.array([161, 155, 84])
    # high_red = np.array([179, 255, 255])
    # red_mask = cv2.inRange(frame, low_red, high_red)
    # lower mask (0-10)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask0 = cv2.inRange(frame, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask1 = cv2.inRange(frame, lower_red, upper_red)

    # join my masks
    red_mask = cv2.bitwise_or(mask0, mask1)
    # red = cv2.bitwise_and(frame, frame, mask = red_mask)

    countRed = np.count_nonzero(red_mask)
    # if countRed > 0:
    # print("Red value: ",countRed)
    if countRed > 10:
        return True
    else:
        return False


def detect_green(frame):
    low_green = np.array([25, 52, 72])
    high_green = np.array([102, 255, 255])

    green_mask = cv2.inRange(frame, low_green, high_green)
    # red = cv2.bitwise_and(frame, frame, mask = red_mask)

    countGreen = np.count_nonzero(green_mask)
    # print("Green value: ",countGreen)
    if countGreen > 110:
        return True
    else:
        return False


def region_of_interest(edges):
    height, width = edges.shape  # extract the height and width of the edges frame
    # make an empty matrix with same dimensions of the edges frame
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height),
        (0,  height/2),
        (width, height/2),
        (width, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)  # fill the polygon with blue color
    cropped_edges = cv2.bitwise_and(edges, mask)
    # cv2.imshow("roi", cropped_edges)
    return cropped_edges


def detect_line_segments(cropped_edges):
    rho = 1
    theta = np.pi / 180
    min_threshold = 10
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=0)
    return line_segments


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0:
        slope = 0.1

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]


def average_slope_intercept(frame, line_segments):
    lane_lines = []

    if line_segments is None:
        # print("no line segment detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3

    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                # print("skipping vertical lines (slope = infinity)")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane
    # all coordinate points are in pixels
    return lane_lines


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):  # line color (B,G,R)
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2),
                         line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2:  # if two lane lines are detected
        # extract left x2 from lane_lines array
        _, _, left_x2, _ = lane_lines[0][0]
        # extract right x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

    elif len(lane_lines) == 1:  # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0:  # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):

    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):

    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


# Variables to be updated each loop
lastTime = 0
lastError = 0

# PWM pin
motor_in = "P9_14"
steering_in = "P9_16"

if __name__ == "__main__":
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    PWM.start(steering_in, 7.5, 50)

    spdinit = 7.5+0.0195

    stop = 0

    need_traffic_light = True
    stop_before_traffic_light = False

    frame_no = -1

    f = open("log.csv", "w")
    f.write("frame no, steering PWM, speed PWM\n")
    while True:
        frame_no += 1

        ret, frame = video.read()
        # resize the frame to speedup image processing
        frame = cv2.resize(frame, (80, 40), cv2.INTER_CUBIC)

        hsv = convert_to_HSV(frame)
        edges = detect_edges(hsv)

        # stop at traffic light
        if need_traffic_light and detect_red(hsv):
            # print("Traffic Red!")
            PWM.start(motor_in, 7.5, 50)
            stop_before_traffic_light = True
            need_traffic_light = False

        # wait unitil the traffic lights turn green, and then keep going
        if stop_before_traffic_light:
            if detect_green(hsv):
                # print("see green")
                PWM.start(motor_in, 7.92, 50)
                stop_before_traffic_light = False
            else:
                # print("not see green")
                continue

        # detect STOP region
        if detect_red(hsv):
            stop += 1
            if stop == 1:
                # print("Stop Red!")
                PWM.start("P9_14", 7.5, 50)
                time.sleep(2.5)
                PWM.start("P9_14", 7.92, 50)
            # skip some frames not to detect STOP regions,
            # so that our car can pass the region without stoppoing again
            if stop == 100:
                stop = 0

        roi = region_of_interest(edges)
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame, line_segments)
        lane_lines_image = display_lines(frame, lane_lines)
        steering_angle = get_steering_angle(frame, lane_lines)
        heading_image = display_heading_line(lane_lines_image, steering_angle)
        #cv2.imshow("heading_img", heading_image)

        deviation = steering_angle - 90  # equivalent to angle_to_mid_deg variable
        error = abs(deviation)

        # do not steer if there is a 15-degree error range
        if deviation < 15 and deviation > -15:
            deviation = 0
            error = 0
            PWM.start(steering_in, 7.5, 50)
            PWM.start(motor_in, spdinit + 0.4175, 50)
            f.write(str(frame_no) + "," + str(7.5) +
                    "," + str(spdinit + 0.4175)+"\n")
        # otherwise, map the deviation to the steering PWM input linearly
        else:
            out = 7.3 - deviation * 5 / 80 # steering PWM
            PWM.start(steering_in, out, 50)
            # if the turing angle is too large, then enlarge the speed PWM to boost its power
            if out > 9 and out < 6:
                PWM.start(motor_in, spdinit+0.4705, 50)
                f.write(str(frame_no) + "," + str(out) +
                        "," + str(spdinit + 0.4705)+"\n")
                f.flush()
            # if the turn is not too sharp, just enlarge the speed PWM a little bit
            else:
                PWM.start(motor_in, spdinit+0.452, 50)
                f.write(str(frame_no) + "," + str(out) +
                        "," + str(spdinit + 0.452)+"\n")
                f.flush()

            lastError = error
            lastTime = time.time()

        key = cv2.waitKey(1)
        if key == 27:
            PWM.start(motor_in, 7.5, 50)
            PWM.start(steering_in, 7.5, 50)
            break
    
    f.close()
    video.release()
    cv2.destroyAllWindows()
