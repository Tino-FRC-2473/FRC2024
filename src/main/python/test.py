import cv2
from detector import Detector
import numpy as np
import os
import time

cap = cv2.VideoCapture(0)
counter = 0

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

    orange_mask = d.detectOrange(frame_single_channel, threshold=100)
    contour = d.find_largest_orange_contour(orange_mask)
    #cv2.drawContours(frame, contour, 0, [255, 0, 0], 2)
    if contour is not None and d.contour_is_note(contour):
        print("ellipse error being thrown AFTER contour detected")
        cv2.ellipse(frame, cv2.fitEllipse(contour), (255, 0, 255), 2)

    # TODO: ALTER THRESHOLD VALS HERE - define orange based on intensity thresholds? (exposure, brightness..)
    cv2.imshow("masked stream", orange_mask)

    key = cv2.waitKey(1)

    filename = f"frame_"+ str(counter) + ".jpg"  # Customize filename format if needed
    counter +=1
    
    if key == ord('q'):  # Quit if 'q' key is pressed

        break




   
