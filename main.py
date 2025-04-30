import cv2
import numpy as np
import time
import math
from smbus2 import SMBus

# === MCP4728 Driver for Raspberry Pi ===
class MCP4728:
    def __init__(self, i2c_bus=1, address=0x64):
        self.bus = SMBus(i2c_bus)
        self.address = address

    def set_channel_value(self, channel, value):
        if channel not in ['a', 'b', 'c', 'd']:
            raise ValueError("Invalid channel")

        # Convert 16-bit to 12-bit
        value = max(0, min(value, 65535))
        value12 = value >> 4

        command_map = {'a': 0x40, 'b': 0x50, 'c': 0x60, 'd': 0x70}
        cmd = command_map[channel]

        msb = (value12 >> 4) & 0xFF
        lsb = (value12 << 4) & 0xFF

        self.bus.write_i2c_block_data(self.address, cmd, [msb, lsb])

# === Initialization ===
mcp4728 = MCP4728()
video = cv2.VideoCapture(0)

lastError = 0
integral = 0
kP = 100
kI = 0
kD = 100
baseSpeed = 30000

while True:
    ret, frame = video.read()
    if not ret:
        continue

    # Crop and preprocess
    roi = frame[350:480, 0:640]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, threshold = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

    # Contour detection
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw guide lines
    cv2.line(roi, (320, 0), (320, 130), (255, 0, 0), 2)
    cv2.line(roi, (0, 50), (640, 50), (255, 0, 0), 2)

    # Select best contour
    bestContour = None
    maxArea = 0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w * h > maxArea and h > 30:
            bestContour = contour
            maxArea = w * h

    # PID control
    if bestContour is not None:
        M = cv2.moments(bestContour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            error = 320 - cx
            integral += error
            derivative = error - lastError
            currentSpeed = kP * error + kI * integral + kD * derivative
            lastError = error

            # Constrain and normalize
            currentSpeed = int(max(-65535, min(currentSpeed, 65535)))
            steering = 32768 + currentSpeed
            steering = max(0, min(steering, 65535))

            # Apply speed and steering
            mcp4728.set_channel_value('c', int(steering))
            mcp4728.set_channel_value('d', int(baseSpeed))

            # Draw target point
            cv2.circle(roi, (cx, cy), 5, (255, 255, 255), -1)

    cv2.imshow("roi", roi)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
video.release()
cv2.destroyAllWindows()
