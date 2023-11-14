import cv2
import numpy as np
import open3d as o3d
# import pyrealsense2 as rs
from frankapy import FrankaArm
import robomail.vision as vis

# checkerboard_size = (6, 9)
checkerboard_size = (5,8)
upper_right_ee_offset = np.array([0.025, 0.06, 0.03])
# upper_left_ee_offset = np.array([0.028, -0.08, 0.03]) # for the vertical checkerboard
upper_left_ee_offset = np.array([0.025, -0.08, -0.03]) # for the horizontal checkerboard


# define data parameters
cam_num = 4
exp_num = 0
save_path = "Calibration_Data/Horizontal_Board/Cam" + str(cam_num)

# initialize camera
cam = vis.CameraClass(cam_num)

# define robot
fa = FrankaArm()

# # FOR THE VERTICAL CHECKERBOARD get 3D points in ee frame
# ee_points = []
# square = 0.03 # [cm]
# # iterating from left to right: [y]
# for i in range(checkerboard_size[0]):
#     # iterating from top to bottom: [z]
#     for j in range(checkerboard_size[1]):
#         x = upper_left_ee_offset[0]
#         y = upper_left_ee_offset[1] 
#         z = upper_left_ee_offset[2] 

#         position = np.array([x, y + square*(i+1), z - square*(j+1)])
#         ee_points.append(position)

# FOR THE HORIZONTAL CHECKERBOARD get 3D points in ee frame
ee_points = []
square = 0.03 # [cm]
# iterating from left to right: [y]
for i in reversed(range(checkerboard_size[0])):
    # iterating from front to back: [x]
    for j in reversed(range(checkerboard_size[1])):
        x = upper_left_ee_offset[0]
        y = upper_left_ee_offset[1] 
        z = upper_left_ee_offset[2] 

        position = np.array([x + square*(j+1), y + square*(i+1), z])
        ee_points.append(position)

ee_points = np.array(ee_points)
print("EE Points: ", ee_points.shape)

# single image loop
pose = fa.get_pose()
rot = pose.rotation
t = np.expand_dims(pose.translation, axis=1)
world_transform = np.hstack((rot, t))
world_transform = np.vstack((world_transform, np.array([0,0,0,1])))

pointcloud = o3d.geometry.PointCloud()
pointcloud.points = o3d.utility.Vector3dVector(ee_points)
pointcloud.transform(world_transform)
# o3d.visualization.draw_geometries([pointcloud])
world_points = np.asarray(pointcloud.points)
print("World points: ", world_points.shape)

# corner detection
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

img, depth, pointcloud, verts = cam.get_next_frame()
bw_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# find checkerboard corners 
ret, corners = cv2.findChessboardCorners(bw_image, checkerboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

if ret == True:
    corners_refined = cv2.cornerSubPix(bw_image, corners, (11, 11), (-1, -1), criteria)

    # visualize corners in the order they are detected
    cXs = []
    cYs = []
    for corner in corners:
        cX = int(corner[0][0])
        cY = int(corner[0][1])
        cXs.append(cX)
        cYs.append(cY)
        cv2.circle(bw_image, (cX, cY), 5, (0,0, 255), -1)
        cv2.imshow('image', bw_image)
        cv2.waitKey(220)
    
    # event = keyboard.read_event(suppress=True)
    # if event.name == "y":
    #     print("Yes")
    input("Press Enter to continue...")

    cam_points = verts[cYs, cXs].reshape(-1,3)
    print("Cam Points: ", cam_points.shape)

    # save cam_points
    np.save(save_path + "/cam" + str(cam_num) + "_camframe_" + str(exp_num) + ".npy", cam_points)
    np.save(save_path + "/cam" + str(cam_num) + "_robframe_" + str(exp_num) + ".npy", world_points)