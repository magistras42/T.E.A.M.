import cv2
import math
import numpy as np
import video_processing as vp
import time
import torch
import matplotlib as plt
from ultralytics import YOLO

frames = []
periods = []

try:
    # Initialize video, process 320 x 320 resoution.
    video = vp.init_video()
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

    # Object recognition model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    model.eval()

    t = time.time()

    # Main loop for video processing code.
    while True:
        # Read video frame.
        ret, frame = video.read()
        if not ret:
            break

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model(img)

        # Draw detections
        for x1, y1, x2, y2, conf, cls in results.xyxy[0]:
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            label = f"{model.names[int(cls)]} {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Store frame in memory
        frames.append(frame.copy())

        # Display the frame
        cv2.imshow("YOLOv5", frame)

        # Framerate tracking
        nt = time.time()
        td = nt - t
        t = nt
        periods.append(td)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()

# handle keyboard shutdown gracefully
except KeyboardInterrupt or Exception as e:
    # Clean up video processing resources.
    video.release()
    cv2.destroyAllWindows()

    if len(periods) > 1:
        periods.pop(0)

    # Calculate average framerate.
    ave_period = sum(periods) / len(periods)
    ave_fps = 1 / ave_period
    print("Average framerate: {:.2f}".format(ave_fps))

    # Save video using average framerate
    if frames:
        frame_height, frame_width = frames[0].shape[:2]
        size = (frame_width, frame_height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('output.mp4', fourcc, ave_fps, size)

        # Write out all frames to an mp4 file
        for f in frames:
            out.write(f)

        out.release()
        print("Video saved as output.mp4")
