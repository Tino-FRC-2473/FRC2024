import cv2
from detector import Detector
import numpy as np
import os
import time

cap = cv2.VideoCapture(0)
counter = 0

LOW_THRESHOLD = 100
HIGH_THRESHOLD = 150

while(True):

    ret, frame = cap.read()
    assert ret

    # import matplotlib.pyplot as plt
    # plt.figure()
    # # need a 2D array at least for an image
    # plt.imshow(frame[:,:,0])
    # #plt.imshow(frame)
    # plt.show()

    d = Detector()

    # convert tuple from (height, width, # of channels) to just (height, width)
    frame_single_channel = frame[:,:,0]

    orange_mask = d.detectOrange(frame_single_channel, LOW_THRESHOLD, HIGH_THRESHOLD)
    largest_contour = d.find_largest_orange_contour(orange_mask)
    if len(largest_contour) >= 5:
            print("contour has min 5 points")
    #cv2.drawContours(frame, contour, 0, [255, 0, 0], 2)
    if largest_contour is not None and d.contour_is_note(largest_contour):
        print(d.contour_is_note(largest_contour))
        cv2.ellipse(orange_mask, cv2.fitEllipse(largest_contour), (255, 0, 255), 2)

    # TODO: ALTER THRESHOLD VALS HERE - define orange based on intensity thresholds? (exposure, brightness..)
    cv2.imshow("masked stream", orange_mask)

    key = cv2.waitKey(1)

    filename = f"frame_"+ str(counter) + ".jpg"  # Customize filename format if needed
    counter +=1
    
    if key == ord('q'):  # Quit if 'q' key is pressed

        break




   
