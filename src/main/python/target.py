import cv2
import math


class Target:
    FOV = (50.28, 29.16)
    RES = (1280, 720)
    CAM_HEIGHT = 0.15
    CAM_ANGLE = 0

    def __init__(self, contour, target_type):
        self.contour = contour
        self.target_type = target_type
        self.heights = {"CUBE": 0.24/2, "CONE": 0.33/2}

    def __str__(self):
        return self.target_type + " coords - " + self.contour

    def getType(self):
        return self.target_type

    def getContour(self):
        return self.contour

    def getBoundingRect(self):
        return cv2.boundingRect(self.contour)

    def get_yaw_degrees(self):
        x, y, w, h = self.getBoundingRect()
        center_tag = x + (w/2)
        #print(x)
        center_cam = Target.RES[0]/2
        B = center_tag - center_cam
        A = center_cam
        theta = math.atan(B * math.tan(math.radians(Target.FOV[0] / 2)) / A)
        return math.degrees(theta)

    def get_pitch_degrees(self):
        x, y, w, h = self.getBoundingRect()
        center_tag = y + (h/2)
        center_cam = Target.RES[1]/2
        B = center_cam - center_tag
        A = center_cam
        theta = math.atan(B * math.tan(math.radians(Target.FOV[1] / 2)) / A)
        return math.degrees(theta)

    def get_distance_meters(self):
        height = self.heights[self.getType()]
        return (height - Target.CAM_HEIGHT) / math.tan(math.radians(Target.CAM_ANGLE + self.get_pitch_degrees()))
