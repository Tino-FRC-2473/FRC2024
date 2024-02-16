import cv2
import numpy as np
from apriltag import AprilTag
from vision_input import VisionInput
import time
import threading

class MyThread (threading.Thread):

    def __init__(self, tag_module):
        self.params = None
        self.lock = threading.RLock()
        self.tag_module = tag_module
        super(MyThread, self).__init__()

    def set_params(self, params):  # you can use a proper setter if you want
        with self.lock:
            self.params=params

    def run(self):
        while True:
            p = time.time()
            self.tag_module.estimate_3d_pose(self.params)
            print("loop time: " + str(time.time()-p))
            cv2.imshow('result', self.params[1])
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

RES = (640, 480)

tag_module = AprilTag()
CALIB_DIR = 'cam1_photos'
CALIB_SIZE_METERS = 0.015
CALIB_WIDTH = 9
CALIB_HEIGHT = 9
#tag_module.calibrate(RES, CALIB_DIR, CALIB_SIZE_METERS, CALIB_WIDTH, CALIB_HEIGHT, visualize=True)
#UNCOMMENT ABOVE IF CALIBRATION DATA is not in /calibration_data direcotry

FOV = (50.28, 29.16)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
TAG_LENGTH_METERS = 0.165
tagThread = MyThread(tag_module)
tagThread.set_params((input.getFrame(), input.getFrame().copy(), TAG_LENGTH_METERS))
tagThread.start()


while True:
    frame = input.getFrame()
    annotated_frame = frame.copy()
    params = (frame, annotated_frame, TAG_LENGTH_METERS)
    tagThread.set_params(params)


