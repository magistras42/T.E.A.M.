import cv2
import time
import numpy as np

LOWER_LIMIT_RED1 = [0, 40, 60]
UPPER_LIMIT_RED1 = [10, 255, 255]

LOWER_LIMIT_RED2 = [170,40, 60]
UPPER_LIMIT_RED2 = [180, 255, 255]

# STOP_PERCENT_LOWER = 17
# STOP_PERCENT_UPPER = 20
STOP_PERCENT = 18

def is_red(frame):
    # Convert to HSV color space
    hsv_frm = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # rough region of interest given our camera setup
    x1 = 110
    y1 = 60
    x2 = 210
    y2 = 160
    roi = hsv_frm[y1:y2, x1:x2]

    cv2.imwrite("red.jpg", roi)

    # lower and upper range for the lower red bound
    lower_red1 = np.array(LOWER_LIMIT_RED1, dtype="uint8")
    upper_red1 = np.array(UPPER_LIMIT_RED1, dtype="uint8")

    # lower and upper range for the upper red bound
    lower_red2 = np.array(LOWER_LIMIT_RED2, dtype="uint8")
    upper_red2 = np.array(UPPER_LIMIT_RED2, dtype="uint8")

    # create two masks to capture both ranges of red
    mask1 = cv2.inRange(roi, lower_red1, upper_red1)
    mask2 = cv2.inRange(roi, lower_red2, upper_red2)

    # combine masks
    mask = cv2.bitwise_or(mask1, mask2)

    # apply mask
    output = cv2.bitwise_and(roi, roi, mask=mask)
    
    # save the output image
    cv2.imwrite("redmask.jpg", output)
    #full_red_mask = cv2.bitwise_or(mask1, mask2)

    # calculate what percentage of image falls between color boundaries
    # Calculate red pixel percentage
    #red_pixels = cv2.countNonZero(full_red_mask)
    #total_pixels = roi.shape[0] * roi.shape[1]
    #percent_red = (output / total_pixels) * 100
    
    percent_red = np.count_nonzero(mask) * 100 / np.size(mask)
    
    # return if a greater percentage of the image is red than STOP_PERCENT
    result = STOP_PERCENT < percent_red
    print(percent_red, result)
    return result

cap = cv2.VideoCapture(0)  # or replace with your video file or stream URL
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Show the frame in a window
    cv2.imshow('Camera Feed', frame)
    
    is_red(frame)
    time.sleep(.5)

    # Break on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
