import cv2
from detector import Detector

#Showing output from arducam
# open video0
cap = cv2.VideoCapture(1)
printed = False
threshold = 40
while(True):
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    # set exposure time
    cap.set(cv2.CAP_PROP_EXPOSURE, -12)
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Display the resulting frame
    
    if(not printed):
        print(frame)
        printed = True

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    d = Detector()

    cv2.imshow('frame', d.detectOrange(frame[:,:,0], threshold))

    key = cv2.waitKey(1)
     # Check if 'a' key is pressed
    if key == ord('a'):
        # Pause and allow the user to change the threshold value
        threshold = int(input("Enter a new threshold value: "))
        print("new threshold value: " + str(threshold))
    elif key == ord('q'):  # Quit if 'q' key is pressed
        break




   
