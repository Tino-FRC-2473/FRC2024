import cv2
import numpy as np
from apriltag import AprilTag
from vision_input import VisionInput
import time


tag_module = AprilTag()
# CALIB_DIR = 'calib_images'
# CALIB_SIZE_METERS = 0.0301625
# CALIB_WIDTH = 5
# CALIB_HEIGHT = 7
# tag_module.calibrate(CALIB_DIR, CALIB_SIZE_METERS, CALIB_WIDTH, CALIB_HEIGHT)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry

FOV = (50.28, 29.16)
RES = (640, 480)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)

while True:
    frame = input.getFrame()
    annotated_frame = frame.copy()
    pose_data = tag_module.estimate_3d_pose(frame, annotated_frame)
    print(pose_data)

    cv2.imshow('result', annotated_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    time.sleep(0.02)


