import cv2
import math
import numpy as np
from target import Target
import time

# define orange hsv values (opencv range is h:0-180, s:0-255, v:0-255)
# NOTE: BELOW VALS REPRESENT RED ACTUALLY BC I ONLY HAD A RED SPONGE TO USE 
# LOWER_ORANGE_HSV = np.array([3, 80, 80])
# UPPER_ORANGE_HSV = np.array([6, 255, 255])

# orange hsv values
# LOWER_ORANGE_HSV = np.array([3, 80, 80])
# UPPER_ORANGE_HSV = np.array([6, 255, 255])

# minimum contour area to detect a note
MINIMUM_CONTOUR_AREA = 400
# threshold for a contour to be considered a disk
CONTOUR_DISK_THRESHOLD = 0.9

class Detector:

    def __init__(self):
        pass

    def detectOrange(self, grayscale_image, low_threshold, high_threshold):     
        """NOTE: to threshold with monochrome image (single-channel, grayscale) to create a binary mask, 
                can specify a SINGLE scalar value for lower/upper bounds
                returns: binary image (single-channel, 8-bit)"""
        #return cv2.inRange(grayscale_image, low_threshold, high_threshold)
        return np.where(grayscale_image > low_threshold, 255, 0).astype(np.uint8)    
    
    def find_largest_orange_contour(self, orange_mask: np.ndarray) -> np.ndarray:
        """
        finds the largest contour in the mask
        input: thresholded image (np array)
        output: largest contour (np array)
        """

        # find contours in the mask:  (* ignore second return value *)
        # cv2.RETR_EXTERNAL retrieves external contours only (helps w focusing on outer ring)
        # cv2.CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments and leaves only their end points 
        # input has to be binary image (CV_8UC1)
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # if contours:
        #     # draws everything else it's detecting
        #     for i in range(len(contours)):
        #         if len(contours[i]) >= 5:
        #             print("contours with greater than 5 points")
        #             cv2.drawContours(orange_mask, contours, -1, [0, 255, 0], 1)
        #             # gets the largest contour and draws it on
        #             largest_contour = max(contours, key=cv2.contourArea)
        #             cv2.drawContours(orange_mask, [largest_contour], 0, [255, 0, 0], 2)

        return max(contours, key=cv2.contourArea)

    def get_ellipse(self, contour: np.ndarray):
        """
        checks if the contour is shaped like a note
        input: contour (np array)
        output: if contour is a ring (boolean)
        """
        if len(contour) < 5:
            return False  # Not enough points to fit an ellipse
        
        # makes sure the contour isn't some random small spec of noise
        if cv2.contourArea(contour) < MINIMUM_CONTOUR_AREA:
            return False
        
        # gets the convex hull: smallest convex polygon that can fit around the contour
        contour_hull = cv2.convexHull(contour)
        
        if len(contour_hull) < 5:
            return False  # Not enough points to fit an ellipse

        # fits an ellipse to the hull, and gets its area
        # if len(contour_hull) >= 5:
        #     print("contour has min 5 points")

        #returns a rotated rectangle in which the ellipse can fit 
        ellipse = cv2.fitEllipse(contour_hull)
        return ellipse
     
    def contour_is_note(self, contour: np.ndarray) -> bool:
        """
        checks if the contour is shaped like a note
        input: contour (np array)
        output: if contour is a ring (boolean)
        """
        if len(contour) < 5:
            return False  # Not enough points to fit an ellipse
        
        # makes sure the contour isn't some random small spec of noise
        if cv2.contourArea(contour) < MINIMUM_CONTOUR_AREA:
            return False
        
        # gets the convex hull: smallest convex polygon that can fit around the contour
        contour_hull = cv2.convexHull(contour)
        
        if len(contour_hull) < 5:
            return False  # Not enough points to fit an ellipse

        # fits an ellipse to the hull, and gets its area
        # if len(contour_hull) >= 5:
        #     print("contour has min 5 points")

        #returns a rotated rectangle in which the ellipse can fit 
        ellipse = cv2.fitEllipse(contour_hull)
        print("printing ellipse")
        print(ellipse)
        # area formula: pi * semi-major axis * semi-minor axis
        best_fit_ellipse_area = np.pi * (ellipse[1][0] / 2) * (ellipse[1][1] / 2)


        """compares area of the hull to area of the best-fit ellipse, if the ratio is greater than a certain threshold, 
        returns true & indicates that the contour is likely shaped like a note"""
        return cv2.contourArea(contour_hull) / best_fit_ellipse_area > CONTOUR_DISK_THRESHOLD

    #((150.0, 200.0), (100.0, 50.0), 30.0)
    #In this example:
    #The center of the ellipse is at (150.0, 200.0).
    #The major axis length is 100.0 and the minor axis length is 50.0.
    #The ellipse is rotated by 30.0 degrees.
    def get_yaw_degrees(self, contour):
        ellipse = self.get_ellipse(contour)
        center_tag = ellipse[0][0]
        #print(x)
        center_cam = Target.RES[0]/2
        B = center_tag - center_cam
        A = center_cam
        theta = math.atan(B * math.tan(math.radians(Target.FOV[0] / 2)) / A)
        #print(math.degrees(theta))
        return math.degrees(theta)

    # previous detection code..? not sure what this was for
    # def detectGameElement(self, frame, objectsToDetect: list):

    #     results = dict(zip(objectsToDetect, [None for i in range(len(objectsToDetect))]))
        
    #     for object in objectsToDetect:
          
    #         #The following three functinos edits the mask in order to remove potential discrepencies in the frame
    #         kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (12, 12))
    #         morph = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
    #         #blurred = cv2.GaussianBlur(mask, 5)

            

    #         #The below code runs to detect if there is a ring in the given frame.
    #         if (object == "RING"):
    #             contours, hier = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #             contours = sorted(contours, key=cv2.contourArea)
    #             contours = [contour for contour in contours if contour.size > 1000]
    #             #last detection is supposed be the biggest because of the sorting function above
    #             if (len(contours) > 0):
    #                 tx,ty,tw,th = cv2.boundingRect(contours[len(contours) -1])
    #                 cv2.rectangle(frame, (tx, ty), (tx + tw, ty + th),
    #                                      (0, 0, 255), 2)

    #     if (len(contours) > 0):
    #         results[object] = Target(contours[len(contours) -1], object)
    #         return results
    #     return None
            
        
