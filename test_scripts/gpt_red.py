import cv2
import numpy as np
import time
import logging

# HSV Red color range
LOWER_LIMIT_RED1 = [0, 40, 60]
UPPER_LIMIT_RED1 = [10, 255, 255]
LOWER_LIMIT_RED2 = [170, 40, 60]
UPPER_LIMIT_RED2 = [180, 255, 255]

# Camera resolution
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# ROI: bottom half of the screen
ROI_X1 = 0
ROI_Y1 = FRAME_HEIGHT // 2         # 120
ROI_X2 = FRAME_WIDTH               # 320
ROI_Y2 = FRAME_HEIGHT              # 240

# Threshold for red percentage
STOP_PERCENT = 35

# Debug flags
SAVE_IMAGES = False
DEBUG_LOGGING = True

if DEBUG_LOGGING:
    logging.basicConfig(level=logging.INFO)

def is_red(frame):
    """Detects if red dominates in the region of interest."""
    # Convert to HSV
    hsv_frm = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    roi = hsv_frm[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]

    # Define red masks
    mask1 = cv2.inRange(roi, np.array(LOWER_LIMIT_RED1, dtype="uint8"), np.array(UPPER_LIMIT_RED1, dtype="uint8"))
    mask2 = cv2.inRange(roi, np.array(LOWER_LIMIT_RED2, dtype="uint8"), np.array(UPPER_LIMIT_RED2, dtype="uint8"))
    mask = cv2.bitwise_or(mask1, mask2)

    # Optionally save debug images
    if SAVE_IMAGES:
        timestamp = int(time.time() * 1000)
        cv2.imwrite(f"roi_{timestamp}.jpg", cv2.cvtColor(roi, cv2.COLOR_HSV2BGR))
        masked_roi = cv2.bitwise_and(roi, roi, mask=mask)
        cv2.imwrite(f"redmask_{timestamp}.jpg", cv2.cvtColor(masked_roi, cv2.COLOR_HSV2BGR))

    # Compute red percentage
    red_pixels = np.count_nonzero(mask)
    total_pixels = mask.shape[0] * mask.shape[1]
    percent_red = (red_pixels * 100.0) / total_pixels
    result = percent_red > STOP_PERCENT

    if DEBUG_LOGGING:
        logging.info(f"Red %: {percent_red:.2f} — Trigger: {result}")

    return result

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

print("📷 Press 'q' to quit.")
while True:
    time.sleep(0.5)
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Frame grab failed — Exiting.")
        break

    # Draw ROI box
    cv2.rectangle(frame, (ROI_X1, ROI_Y1), (ROI_X2, ROI_Y2), (0, 255, 0), 2)

    # Check for red in ROI
    is_red(frame)

    # Display the frame
    cv2.imshow("Camera Feed", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()

