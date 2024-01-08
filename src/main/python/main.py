#import cv2
import numpy as np
from detector import Detector
from vision_input import VisionInput
import time
import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("python")
inst.setServerTeam(2473)

FOV = (50.28, 29.16)
RES = (320, 240)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
d = Detector()
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
curr = 0
cnt = 0
while True:
    #print("here1")
    frame = input.getFrame()
    table = inst.getTable("datatable")
    xPub = table.getDoubleTopic("fps_incremented_value").publish()
    xPub.set(frame.sum())

    coneY = table.getDoubleTopic("cone_yaw").publish()
    coneD = table.getDoubleTopic("cone_distance").publish()
    cubeY = table.getDoubleTopic("cube_yaw").publish()
    cubeD = table.getDoubleTopic("cube_distance").publish()

    coneYSub = table.getDoubleTopic("cone_yaw").subscribe(0)
    coneDSub = table.getDoubleTopic("cone_distance").subscribe(0)

    results = d.detectGameElement(np.asarray(frame), ["CUBE", "CONE"])

    for type, target in results.items():
        if target is not None:
            yaw = target.get_yaw_degrees()
            distance = target.get_distance_meters()

            #print("detection: ", time.time() - curr)
            curr = time.time()

            if target.getType() == "CONE":
                coneY.set(yaw)
                coneD.set(distance)
                if cnt % 50 == 0:
                    print(coneYSub.get())
                    print(coneDSub.get())
            elif target.getType() == "CUBE":
                cubeY.set(yaw)
                cubeD.set(distance)
    #print("here")
    cnt = cnt + 1
    time.sleep(0.02)

   
