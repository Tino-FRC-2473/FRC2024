import cv2
from detector import Detector
import numpy as np
import os
import time
import math
from target import Target

cap = cv2.VideoCapture(0)

LOW_THRESHOLD = 80
HIGH_THRESHOLD = 170

while(True):

    ret, frame = cap.read()
    assert ret 
    
    d = Detector()

    # convert tuple from (height, width, # of channels) to just (height, width)
    frame_single_channel = frame[:,:,0]

    orange_mask = d.detectOrange(frame_single_channel, LOW_THRESHOLD, HIGH_THRESHOLD)
    largest_contour = d.find_largest_orange_contour(orange_mask)

    print("yaw degrees" + str(d.get_yaw_degrees(largest_contour)))

    #cv2.drawContours(frame, contour, 0, [255, 0, 0], 2)
    if largest_contour is not None and d.contour_is_note(largest_contour):
        target = Target(largest_contour, "RING")

        yaw = target.get_yaw_degrees()
        dist = target.get_distance_meters()
        print("yaw: " + yaw)
        print("distance: " + dist)

        cv2.ellipse(orange_mask, cv2.fitEllipse(largest_contour), (255, 0, 255), 10) # TRY -1

    # TODO: ALTER THRESHOLD VALS HERE - define orange based on intensity thresholds? (exposure, brightness..)
    cv2.imshow("masked stream", orange_mask)

    key = cv2.waitKey(1)
    if key == ord('q'):  # Quit if 'q' key is pressed
        break
    
cap.release()
cv2.destroyAllWindows()
