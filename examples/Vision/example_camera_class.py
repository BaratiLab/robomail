import cv2
import numpy as np
import robomail.vision as vis

"""
Example script demonstrating how to call camera wrapper to simplify using the cameras.
"""

# initialize the end effector camera
cam1 = vis.CameraClass(1)

# functions you can call to get camera information
cam_intrinsics  = cam1.get_cam_intrinsics()
cam_extrinsics = cam1.get_cam_extrinsics()
cam_number = cam1.get_cam_number()
cam_serial = cam1.get_cam_serial()

# begin loop to get images from camera
while True:
    img, depth, pointcloud, verts = cam1.get_next_frame()

    # CAN PUT ANYTHING HERE TO PROCESS THE IMAGES/DEPTH/POINTCLOUD

    # visualize the camera frames
    cv2.imshow("Image", img)
    cv2.waitKey(1)