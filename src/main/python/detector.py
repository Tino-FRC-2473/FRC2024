import cv2
import math
import numpy as np
from target import Target
import skimage.color


class Detector:

    def __init__(self):
        pass

    def bgr_to_rgb(image):
        return image[:,:,::-1]

    def detectGameElement(self, array, objectsToDetect: list):

        constant = 4.2
        low_threshold = np.array([0.02580477, 0.43139728, 0.34260735])

        # WRITE CODE HERE
        high_threshold = np.array([1.08496632, 1.08496632, 1.26602557])

        frame = array
        results = dict(zip(objectsToDetect, [None for i in range(len(objectsToDetect))]))
        colors = {
            "RING": [low_threshold, high_threshold]
        }
        for object in objectsToDetect:
            #Converts the color format of the frame from BGR to HSV for usage with cv2

            full_test_image = self.bgr_to_rgb(frame)
            hsv_frame = skimage.color.rgb2hsv(full_test_image)

            #Creates a mask which is a frame with only the range of colors inputed shown in black and white
            mask = cv2.inRange(hsv_frame, colors[object][0], colors[object][1])

            print(colors[object][0])

            #The following three functinos edits the mask in order to remove potential discrepencies in the frame
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (12, 12))
            morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.medianBlur(mask, 5)

            # plt.figure()
            # plt.title("Masked foreground")
            mask = np.expand_dims(mask, axis = -1)
            rbg_image = skimage.color.hsv2rgb(hsv_frame)
            # plt.imshow(hsv_frame * (1-mask))


            #The below code runs to detect if there is a ring in the given frame.
            if (object == "RING"):
                contours, hier = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                print(len(contours))
                contours = sorted(contours, key=cv2.contourArea)
                # cnt = None
                # contour = contours[len(contours) -1]
                #last detection is supposed be the biggest because of the sorting function above
                tx,ty,tw,th = cv2.boundingRect(contours[len(contours) -1])
                cv2.rectangle(frame, (tx, ty), (tx + tw, ty + th),
                                         (0, 0, 255), 2)

        while True:
            cv2.imshow("o", frame)
            if cv2.waitKey(0):
                break

        cv2.destroyAllWindows()

        results[object] = Target(contours, object)

        return results

    def detectColoredShape(self, array, rgb_col):
        color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
                          'white': [[180, 18, 255], [0, 0, 231]],
                          'red1': [[180, 255, 255], [159, 50, 70]],
                          'red2': [[9, 255, 255], [0, 50, 70]],
                          'green': [[89, 255, 255], [36, 50, 70]],
                          'blue': [[128, 255, 255], [90, 50, 70]],
                          'yellow': [[35, 255, 255], [25, 50, 70]],
                          'purple': [[158, 255, 255], [129, 50, 70]],
                          'orange': [[24, 255, 255], [10, 50, 70]],
                          'gray': [[180, 18, 230], [0, 0, 40]]}
        frame = array
        results = dict(zip(rgb_col, [None for i in range(len(rgb_col))]))

        #print(rgb_col)

        for object in rgb_col:
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_frame, np.array(color_dict_HSV[object][1]), np.array(color_dict_HSV[object][0]))
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
            morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, hier = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[:2]
            x = 0
            y = 0
            w = 0
            h = 0
            for contour in contours:
                tx, ty, tw, th = cv2.boundingRect(contour)
                #print(tx, ty, tw, th)
                if (tw * th > w * h and not
                   (tx == 0 and ty == 0 and tw == frame.shape[1] and th == frame.shape[0])):
                    x = math.sqrt((1.05 * tx - 0.975)/0.227)
                    y = ty
                    w = tw
                    h = th

            results[object] = [x, y, w, h]
            #print("X: %2d, Y: %2d, W: %2d, H: %2d" % (x, y, w, h))
            while True:
                if cv2.waitKey(0):
                    break
            cv2.destroyAllWindows()

            results[object] = Target([x, y, w, h], object)

        return results

    def detectAprilTag(self, array):
        results = []

        return results
