import cv2
from detector import Detector
import numpy as np
import os
import time

cap = cv2.VideoCapture(0)
counter = 0

while(True):
    # turns off auto exposure (for mac: 0.25 is OFF, 0.75 is on)
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    # set exposure time
    # time.sleep(1)
    #cap.set(cv2.CAP_PROP_EXPOSURE, -12)

    ret, frame = cap.read()
    assert ret

    # reduce noise
    blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

    # import matplotlib.pyplot as plt
    # plt.figure()
    # # need a 2D array at least for an image
    # plt.imshow(frame[:,:,0])
    # #plt.imshow(frame)
    # plt.show()

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    d = Detector()

    # convert tuple from (height, width, # of channels) to just (height, width)
    frame_single_channel = frame[:,:,0]

    contour = d.find_largest_orange_contour(frame_single_channel)
    #cv2.drawContours(frame, contour, 0, [255, 0, 0], 2)
    if contour is not None and d.contour_is_note(contour):
        print("ellipse error being thrown AFTER contour detected")
        cv2.ellipse(frame, cv2.fitEllipse(contour), (255, 0, 255), 2)

    # TODO: ALTER THRESHOLD VALS HERE - define orange based on intensity thresholds? (exposure, brightness..)
    # orange_mask = d.detectOrange(frame_single_channel, threshold=100)
    #cv2.imshow("frame", frame)

    key = cv2.waitKey(1)

    filename = f"frame_"+ str(counter) + ".jpg"  # Customize filename format if needed
    counter +=1
    # Assuming your Desktop path, construct the path to save the image
    # image_path = os.path.join(os.path.expanduser('~'), 'Desktop', filename)
    # cv2.imwrite(image_path, frame)
    
    # print(f"Frame saved successfully to: {image_path}")
    if key == ord('q'):  # Quit if 'q' key is pressed

        break




   
