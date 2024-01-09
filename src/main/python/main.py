from vision_input import VisionInput
from apriltag import AprilTag
import time
import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("python")
inst.setServerTeam(2473)

RES = (640, 480)
input = VisionInput(RES)
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

   
