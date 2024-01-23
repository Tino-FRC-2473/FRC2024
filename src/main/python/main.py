from vision_input import VisionInput
from apriltag import AprilTag
import time
import ntcore
import numpy as np

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("python")
inst.setServerTeam(2473)


FOV = (50.28, 29.16)
RES = (640, 480)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
tag_module = AprilTag()
ARUCO_LENGTH_METERS = 0.165

while True:
    frame = input.getFrame()
    table = inst.getTable("datatable")

    xPub = table.getDoubleTopic("fps_incremented_value").publish()
    xPub.set(frame.sum())

    tagData = tag_module.estimate_3d_pose(frame, frame.copy(), ARUCO_LENGTH_METERS)

    pose_list = [4000 for _ in range(16 * 6)]
    for key, value in tagData.items():
        pose_list[(key - 1) * 6 : (key * 6)] = np.concatenate((value[0].flatten(), value[1].flatten()), axis=0).tolist()
    
    tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
    tagDataPub.set(pose_list)
    
    time.sleep(0.02)

   
