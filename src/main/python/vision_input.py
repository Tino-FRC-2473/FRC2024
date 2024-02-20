import cv2
import numpy as np
import math
from target import Target
import time


class VisionInput:
   
    def __init__(self, fov, res: tuple, height, angle):
        self.stream = 0
        self.w = res[0]
        self.h = res[1]
        self.cap = None
        self.cap1 = cv2.VideoCapture(0)
        self.cap2 = cv2.VideoCapture(2)
        Target.FOV = fov
        Target.RES = res
        Target.CAM_HEIGHT = height
        Target.CAM_ANGLE = angle

    def calibrate(self):
        pass
    
    def changeStream(self, stream):
        self.stream = stream
        self.cap = cv2.VideoCapture(self.stream)

    def getFrame(self, stream):
        if(stream == 2):
            self.cap = self.cap1
        else:
            self.cap = self.cap2
        if not self.cap.isOpened():
            print("cannot open cam")

        time.sleep(0.02)
        ret, frame = self.cap.read()

        if not ret:
            print('frame malf')
            return

        fr = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_AREA)
        return fr

    def getGrayFrame(self):
        return cv2.cvtColor(self.getFrame(self), cv2.COLOR_BGR2GRAY)