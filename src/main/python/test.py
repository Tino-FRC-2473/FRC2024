import cv2
from detector import Detector
import numpy as np
import os

#Showing output from arducam
# open video0
cap = cv2.VideoCapture(1)

threshold = 40
counter = 0

while(True):
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    # set exposure time
    cap.set(cv2.CAP_PROP_EXPOSURE, -12)
    # Capture frame-by-frame
    ret, frame = cap.read()

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    d = Detector()
    frame = d.detectOrange(frame[:,:,0], threshold)
    cv2.imshow('frame', frame)

    results = d.detectGameElement(np.asarray(frame), ["RING"])
    print(results)
    key = cv2.waitKey(1)
    # Check if 'a' key is pressed
    #automated way of changing the values of threshold
    '''
    if key == ord('a'):
        # Pause and allow the user to change the threshold value
        threshold = int(input("Enter a new threshold value: "))
        print("new threshold value: " + str(threshold))
    elif key == ord('q'):  # Quit if 'q' key is pressed
        break
    '''

    filename = f"frame_"+ str(counter) + ".jpg"  # Customize filename format if needed
    counter +=1
    # Assuming your Desktop path, construct the path to save the image
    image_path = os.path.join(os.path.expanduser('~'), 'Desktop', filename)
    cv2.imwrite(image_path, frame)
    
    print(f"Frame saved successfully to: {image_path}")
    if key == ord('q'):  # Quit if 'q' key is pressed

        break




   
