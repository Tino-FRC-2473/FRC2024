from vision_input import VisionInput
from apriltag import AprilTag
import time
import ntcore
import numpy as np
import cv2

FOV = (50.28, 29.16)
RES = (640 , 480)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
tag_module = AprilTag()
ARUCO_LENGTH_METERS = 0.165

while True:
    p = time.time()
    try: 
        frame = input.getFrame()

        annotated_frame = frame.copy()
        vision_pose = tag_module.estimate_3d_pose(frame, annotated_frame, ARUCO_LENGTH_METERS)
        annotated_frame = cv2.resize(annotated_frame, (320,240))
        
        res = vision_pose.tolist()
        print(res)

        cv2.imshow('result', annotated_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        time.sleep(0.02)
    except KeyboardInterrupt:
        print("keyboard interrupt")
        input.close()
        break
    # except Exception as error:
    #     print("An exception occurred:", error)
    print('Loop time: ' + str(time.time()-p))