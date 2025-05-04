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
    video = vp.init_video()
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

    # Object recognition model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    model.eval()

    t = time.time()

    while True:
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

except KeyboardInterrupt or Exception as e:
    video.release()
    cv2.destroyAllWindows()

    if len(periods) > 1:
        periods.pop(0)

    ave_period = sum(periods) / len(periods)
    ave_fps = 1 / ave_period
    print("Average framerate: {:.2f}".format(ave_fps))

    # Save video using average framerate
    if frames:
        frame_height, frame_width = frames[0].shape[:2]
        size = (frame_width, frame_height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('output.mp4', fourcc, ave_fps, size)

        for f in frames:
            out.write(f)

        out.release()
        print("Video saved as output.mp4")

"""
import cv2
import math
import numpy as np
import video_processing as vp
import time
import torch
import matplotlib as plt
from ultralytics import YOLO

periods = []

try:
    video = vp.init_video()
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

    # Get the frames per second of the video input
    fps = video.get(cv2.CAP_PROP_FPS) or 20.0  # fallback if FPS not available

    # Get frame size
    frame_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_size = (frame_width, frame_height)

    # Set up video writer
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # or 'MP4V' for .mp4
    out = cv2.VideoWriter('output.avi', fourcc, fps, frame_size)

    # Object recognition model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    model.eval()

    t = time.time()

    while True:
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

        # Show and save the frame
        cv2.imshow("YOLOv5", frame)
        out.write(frame)

        # Framerate tracking
        nt = time.time()
        td = nt - t
        t = nt
        periods.append(td)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video.release()
    out.release()
    cv2.destroyAllWindows()

except KeyboardInterrupt or Exception:
    video.release()
    out.release()
    cv2.destroyAllWindows()
    # Set your desired framerate (e.g., 15.0, 30.0)
    desired_fps = 15.0

    # Frame size
    frame_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_size = (frame_width, frame_height)

    # Set up video writer for MP4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4
    out = cv2.VideoWriter('output.mp4', fourcc, desired_fps, frame_size)
    periods.pop(0)
    ave_period = sum(periods) / len(periods)
    freq = 1 / ave_period
    print("Average framerate: ", freq)

"""
