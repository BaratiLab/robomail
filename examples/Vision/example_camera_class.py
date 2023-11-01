import numpy as np
import robomail.vision as vis

"""
Example script demonstrating how to call camera wrapper to simplify using the cameras.
"""

# initialize the end effector camera
cam1 = vis.CameraClass(1)
cam_intrinsics  = cam1._get_real

# begin loop to get images from camera
while True:
    img, depth, pointcloud, verts = cam1._get_next_frame()
