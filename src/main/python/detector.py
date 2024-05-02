import cv2
import math
import numpy as np
from target import Target
import skimage.color
import time

class Detector:

    def __init__(self):
        pass

    def bgr_to_rgb(self, image):
        return image[:,:,::-1]

    def detectGameElement(self, frame, objectsToDetect: list):

        results = dict(zip(objectsToDetect, [None for i in range(len(objectsToDetect))]))
        
        for object in objectsToDetect:
          
            #The following three functinos edits the mask in order to remove potential discrepencies in the frame
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (12, 12))
            morph = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
            # mask = cv2.medianBlur(mask, 5)

            

            #The below code runs to detect if there is a ring in the given frame.
            if (object == "RING"):
                contours, hier = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                contours = sorted(contours, key=cv2.contourArea)
                contours = [contour for contour in contours if contour.size > 1000]
                #last detection is supposed be the biggest because of the sorting function above
                if (len(contours) > 0):
                    tx,ty,tw,th = cv2.boundingRect(contours[len(contours) -1])
                    cv2.rectangle(frame, (tx, ty), (tx + tw, ty + th),
                                         (0, 0, 255), 2)

        if (len(contours) > 0):
            results[object] = Target(contours[len(contours) -1], object)
            return results
        return None
        
    
    def detectOrange(self, frame, threshold):
        return np.where(frame > threshold, 255, frame)
        
