import cv2
import numpy as np

"""interactive tool for finding appropriate HSV color range to detect orange objects via a video stream
    key: green represents a contour that has been found, blue represents the biggest contour
    - try to get it so contours only appear around notes"""

# creates window to adjust the lower and upper bounds
cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)  # Use WINDOW_NORMAL to allow resizing
cv2.resizeWindow("Trackbars", 600, 300)  # Set the size of the window (width, height)

# creates trackbars
cv2.createTrackbar("Hue Lower", "Trackbars", 1, 179, lambda x: None)
cv2.createTrackbar("Saturation Lower", "Trackbars", 80, 255, lambda x: None)
cv2.createTrackbar("Value Lower", "Trackbars", 130, 255, lambda x: None)
cv2.createTrackbar("Hue Upper", "Trackbars", 6, 179, lambda x: None)
cv2.createTrackbar("Saturation Upper", "Trackbars", 255, 255, lambda x: None)
cv2.createTrackbar("Value Upper", "Trackbars", 255, 255, lambda x: None)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if ret:
        # convert frame from BGR to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # get current trackbar positions
        hue_lower = cv2.getTrackbarPos("Hue Lower", "Trackbars")
        saturation_lower = cv2.getTrackbarPos("Saturation Lower", "Trackbars")
        value_lower = cv2.getTrackbarPos("Value Lower", "Trackbars")
        hue_upper = cv2.getTrackbarPos("Hue Upper", "Trackbars")
        saturation_upper = cv2.getTrackbarPos("Saturation Upper", "Trackbars")
        value_upper = cv2.getTrackbarPos("Value Upper", "Trackbars")

        # define lower and upper bounds for orange color in HSV
        lower_orange = np.array([hue_lower, saturation_lower, value_lower])
        upper_orange = np.array([hue_upper, saturation_upper, value_upper])

        # threshold the HSV image to get only orange colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # find the largest contour (clump) of orange pixels
        if contours:
            # draws everything else it's detecting
            cv2.drawContours(frame, contours, -1, [0, 255, 0], 1)
            # gets the largest contour and draws it on
            largest_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(frame, [largest_contour], 0, [255, 0, 0], 2)

        cv2.imshow("frame", frame)

        # break when 'q' pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("Error: Unable to capture frame")
        break

cap.release()
cv2.destroyAllWindows()