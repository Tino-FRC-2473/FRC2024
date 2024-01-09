from vision_input import VisionInput
from apriltag import AprilTag
import time
import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("python")
inst.setServerTeam(2473)


FOV = (50.28, 29.16)
RES = (640, 480)
CAM_HEIGHT = 0.4
CAM_ANGLE = -15
input = VisionInput(FOV, RES, CAM_HEIGHT, CAM_ANGLE)
tag_module = AprilTag()
while True:
    frame = input.getFrame()
    table = inst.getTable("datatable")

    xPub = table.getDoubleTopic("fps_incremented_value").publish()
    xPub.set(frame.sum())

    tagData = tag_module.estimate_3d_pose(frame, frame.copy())
    tagDataPub = table.getDoubleArrayTopic("april_tag_data").publish()
    tagDataPub.set()
    
    time.sleep(0.02)

   
