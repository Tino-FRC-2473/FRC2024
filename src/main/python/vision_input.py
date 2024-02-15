import cv2
import numpy as np
import math
import time
import pdb


class VisionInput:
    def __init__(self, fov, res: tuple, cam_height, cam_angle):
        try:
            self.fov = fov
            self.w = res[0]
            self.h = res[1]
            self.cam_h = cam_height
            self.cam_a = cam_angle
            print("initializing video device 0")
            self.cap = cv2.VideoCapture(0)
            print("created VideoCapture")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
        except Exception as error:
            self.close()

    def getFrame(self):
        if not self.cap.isOpened():
            print("cannot open cam")
        ret, fr = self.cap.read()
        if not ret:
            print('frame malf')
        return fr


    def close(self):
        self.cap.release()
