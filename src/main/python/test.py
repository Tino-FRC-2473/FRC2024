#import cv2
#import numpy as np
from apriltag import AprilTag
from vision_input import VisionInput
import time

RES = (640, 480)

tag_module = AprilTag()
CALIB_DIR = 'cam1_photos'
CALIB_SIZE_METERS = 0.015
CALIB_WIDTH = 9
CALIB_HEIGHT = 9
tag_module.calibrate(RES, CALIB_DIR, CALIB_SIZE_METERS, CALIB_WIDTH, CALIB_HEIGHT, visualize=True)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry

FOV = (50.28, 29.16)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)

while True:
    frame = input.getFrame()
    annotated_frame = frame.copy()
    m = time.time()
    pose_data = tag_module.estimate_3d_pose(frame, annotated_frame)
    print(time.time() - m)
    print(pose_data)

   # cv2.imshow('result', annotated_frame)
   # key = cv2.waitKey(1) & 0xFF
   # if key == ord('q'):
   #     break
    time.sleep(0.02)


