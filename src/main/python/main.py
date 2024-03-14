import cv2
import numpy as np
from detector import Detector
from vision_input import VisionInput
import time
import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("python")
inst.setServerTeam(2473)

FOV = (50.28, 29.16)
RES = (640, 480)
CAM_HEIGHT = 0.7493
CAM_ANGLE = 50
d = Detector()
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
curr = 0
cnt = 0
while True:
    
    try:
        frame = input.getFrame()
        table = inst.getTable("datatable")

        noteY = table.getDoubleTopic("note_yaw").publish()
        noteD = table.getDoubleTopic("note_distance").publish()

        p = time.time()
        results = d.detectGameElement(np.asarray(frame), ["RING"])
        print("detection time: ", str(time.time() - p))

        if results is not None:
            for type, target in results.items():
                if target is not None:
                    yaw = target.get_yaw_degrees()
                    distance = target.get_distance_meters()
                    pitch = target.get_pitch_degrees()
                    print("yaw: ", yaw)
                    print("distance: ", distance)
                    print("pitch: ", pitch)

                    noteY.set(yaw)
                    noteD.set(distance)
                    curr = time.time()
        else:
            print("no targets")

        cv2.imshow('result', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        time.sleep(0.02)
    except KeyboardInterrupt:
        print("keyboard interrupt")
        input.close()
        break 
    except Exception as error:
        input.close()
        print("An exception occurred:", error)
    print('Loop time: ' + str(time.time()-p))

   
