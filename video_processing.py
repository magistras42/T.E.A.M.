import cv2
import math
import numpy as np

# Camera resolution
FRAME_WIDTH = 320
FRAME_HEIGHT = 176

# HSV Blue color range
LOWER_LIMIT_BLUE = [40, 100, 80]
UPPER_LIMIT_BLUE = [150, 255, 255]

# HSV Red color range
LOWER_LIMIT_RED1 = [0, 40, 60]
UPPER_LIMIT_RED1 = [10, 255, 255]
LOWER_LIMIT_RED2 = [170,40, 60]
UPPER_LIMIT_RED2 = [180, 255, 255]

# ROI for stop detection
ROI_X1 = 0
ROI_Y1 = FRAME_HEIGHT // 2
ROI_X2 = FRAME_WIDTH
ROI_Y2 = FRAME_HEIGHT

# Red threshold for stops
STOP_PERCENT = 30

"""
Initialize video capture.
"""
def init_video():
    video = cv2.VideoCapture(0)
    # 320 x 176 resolution
    video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT,176)
    return video

"""
Convert a camera frame to HSV format.
"""
def convert_to_HSV(frame):
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#   cv2.imshow("HSV",hsv)
  return hsv

"""
Detect blue edges.
"""
def detect_edges(hsv):
    lower_blue = np.array(LOWER_LIMIT_BLUE, dtype = "uint8") # lower limit of blue color
    upper_blue = np.array(UPPER_LIMIT_BLUE, dtype="uint8") # upper limit of blue color
    mask = cv2.inRange(hsv, lower_blue, upper_blue) # this mask will filter out everything but blue
    # cv2.imshow("lanes", mask)
    
    # detect edges
    edges = cv2.Canny(mask, 50, 100) 
    # cv2.imshow("edges",edges)
    return edges

"""
Draw a region of interest for lane lines and crop edges.
"""
def region_of_interest(edges):
    height, width = edges.shape # extract the height and width of the edges frame
    mask = np.zeros_like(edges) # make an empty matrix with same dimensions of the edges frame

    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height), 
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue color 
    cropped_edges = cv2.bitwise_and(edges, mask) 
    # cv2.imshow("roi",cropped_edges)
    return cropped_edges

"""
Given cropped edges, draw detected lane lines.
"""
def detect_line_segments(cropped_edges):
    # Parameters for line detection
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=10, maxLineGap=2)
    # print(line_segments)
    return line_segments

"""
Calculate lane lines depending on bottom half of screen (close to car).
"""
def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down
    # Avoid divide by 0 errors later.
    if slope == 0:
        slope = 0.1
    # calculate x coordinates
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

"""
Find the average slope intercept of the line segments.
"""
def average_slope_intercept(frame, line_segments):
    lane_lines = []

    if line_segments is None:
        # print("no line segment detected")
        return lane_lines
    # Get height and width from frame
    height, width,_ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/8
    # Calculate boundary regions
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    # Iterate through the line segments to calculate the average slope
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            # Ignore vertical lines
            if x1 == x2:
                # print("skipping vertical lines (slope = infinity)")
                continue
            # Calculate lines
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            # Add to appropriate fit array
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
    # Calculate average for left lane fit
    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))
    # Calculate average for right lane fit
    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane
    # all coordinate points are in pixels
    return lane_lines

"""
Display the detected lines on the screen.
"""
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6): # line color (B,G,R)
    line_image = np.zeros_like(frame)
    # Display each of the lines in the provided color.
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

"""
Calculate steering angle for given lane lanes.
"""
def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

    elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)
    # Convert steering angle
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle

"""
Display the direction car will be moving in.
"""
def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    # Create a new image for drawing
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    
    # Convert steering angle
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    # Draw in given color.
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

"""
Calculate the steering angle for a given frame by correcting cur_steering_angle.
"""
def calculate_steering_angle(frame, cur_steering_angle):
        #ret, frame = video.read()
        # frame = cv2.flip(frame,-1)
	# Convert to HSV format and process image
        hsv = convert_to_HSV(frame)
        edges = detect_edges(hsv)
	# Draw ROI
        roi = region_of_interest(edges)
	# Calculate and draw line segments.
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame,line_segments)
        lane_lines_image = display_lines(frame,lane_lines)
	# Calculate steering lines.
        steering_angle = get_steering_angle(frame, lane_lines)
        heading_image = display_heading_line(lane_lines_image,steering_angle)
        heading_image = display_heading_line(heading_image,cur_steering_angle,line_color=(255,0,0))

        # FIXME: Comment out?
        cv2.imshow("angle", heading_image)

        return steering_angle

"""
Detect stop paper.
"""
def is_red(frame):
    # Convert to HSV
    hsv_frm = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = hsv_frm[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]

    # Define red masks
    mask1 = cv2.inRange(roi, np.array(LOWER_LIMIT_RED1, dtype="uint8"), np.array(UPPER_LIMIT_RED1, dtype="uint8"))
    mask2 = cv2.inRange(roi, np.array(LOWER_LIMIT_RED2, dtype="uint8"), np.array(UPPER_LIMIT_RED2, dtype="uint8"))
    mask = cv2.bitwise_or(mask1, mask2)

    # Compute red percentage
    red_pixels = np.count_nonzero(mask)
    total_pixels = mask.shape[0] * mask.shape[1]
    percent_red = (red_pixels * 100.0) / total_pixels
    result = percent_red >= STOP_PERCENT
    # Display result
    if (result):
        print(percent_red)

    return result
