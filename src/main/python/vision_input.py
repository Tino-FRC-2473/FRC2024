import cv2
import numpy as np
import math
import time


class VisionInput:
    def __init__(self, fov, res: tuple, cam_height, cam_angle):
        self.fov = fov
        self.w = res[0]
        self.h = res[1]
        self.cam_h = cam_height
        self.cam_a = cam_angle
        self.cap = cv2.VideoCapture(0)


    def getFrame(self):
        if not self.cap.isOpened():
            print("cannot open cam")
        ret, frame = self.cap.read()

        if not ret:
            print('frame malf')
        exit

        fr = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_AREA)
        return fr


    def close(self):
        self.cap.release()
