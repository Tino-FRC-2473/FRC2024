import cv2
import numpy as np
import math
from target import Target
import time


class VisionInput:
    def __init__(self, fov, res: tuple, height, angle):
        self.w = res[0]
        self.h = res[1]
        self.cap = cv2.VideoCapture(0)
        Target.FOV = fov
        Target.RES = res
        Target.CAM_HEIGHT = height
        Target.CAM_ANGLE = angle

    def calibrate(self):
        pass

    def getFrame(self):
        if not self.cap.isOpened():
            print("cannot open cam")

        time.sleep(0.02)
        ret, frame = self.cap.read()

        if not ret:
            print('frame malf')
        exit

        fr = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_AREA)
        return fr

    def getGrayFrame(self):
        return cv2.cvtColor(self.getFrame(self), cv2.COLOR_BGR2GRAY)
