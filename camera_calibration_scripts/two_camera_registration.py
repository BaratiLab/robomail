import time
import numpy as np
import open3d as o3d
import robomail.vision as vis

"""
This script is the working calibration code to get the transformation matrix between two cameras. Detailed instructions
for how to get the best calibration possible using this script are outlined further below. Our current camera calibration 
protocol is to use two_camera_registration.py to get the transformation matrix between cameras 2/3, 3/4 and 5/4. Then, we
use INSERT_SCRIPT_NAME and a calibration board to find the transformation matrix between camera 4 and the world frame.

GENERAL CALIBRATION INSTRUCTIONS FOR INTER-CAMERA TRANSFORMS:
(1) Place the elevated stage onto the robot workspace and optionally place an asymmetrical calibration object (such as a 
    stack of foam blocks).
(2) Place a solid white poster board slanted in the background of the scene. This provides an additional plane and assists
    the RANSAC algorithm in providing a quality coarse alignment.
(3) Run the two_camera_registration.py script and adjust the voxel size and background removal radius depending on 
    calibration quality, as these are the main parameters we can adjust depending on camera views. More information on
    parameters that were successful for previous calibration runs are provided below.

CALIBRATION PARAMETER NOTES:
(Cam 2/3) 
    - lean the white posterboard against camera 5 at approx a 60 degree angle
    - voxel size: 0.005
    - no background removal
    - ensure calibration object facing both cameras with the same face as much as possible

(Cam 3/4)
    - lean the white posterboard against camera 5, protruding into workspace a bit at approx a 60 degree angle
    - voxel size: 0.01
    - background removal radius: 1.65m
    - ensure calibration object facing both cameras with the same face as much as possible

(Cam 5/4)
    - lean the white posterboard against cameras 2/3 
    - voxel size: 0.018
    - background removal radius: 1.65m
    - ensure calibration object facing both cameras with the same face as much as possible
"""

cama = 2
camb = 3

# initialize the cameras
camera_a = vis.CameraClass(cama)
camera_b = vis.CameraClass(camb)

# initialize the 3D vision code
pcl_vis = vis.Vision3D()

# get the point clouds
_, _, pca, _ = camera_a._get_next_frame()
_, _, pcb, _ = camera_b._get_next_frame()

# combine into single pointcloud
pcda = o3d.geometry.PointCloud()
pcda.points = pca.points
pcda.colors = pca.colors

pcdb = o3d.geometry.PointCloud()
pcdb.points = pcb.points
pcdb.colors = pcb.colors

# remove the background
pcda = pcl_vis.remove_background(pcda, radius=0.9)
pcdb = pcl_vis.remove_background(pcdb, radius=0.9)

# collect addition point clouds to combine
for i in range(9):
    _, _, pca, _ = camera_a._get_next_frame()
    _, _, pcb, _ = camera_b._get_next_frame()
    
    # combine into single pointcloud
    pcda.points.extend(pca.points)
    pcda.colors.extend(pca.colors)
    pcdb.points.extend(pcb.points)
    pcdb.colors.extend(pcb.colors)

pcda, ind = pcda.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
pcdb, ind = pcdb.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# remove the background
pcda = pcl_vis.remove_background(pcda, radius=1.65) # 0.9
pcdb = pcl_vis.remove_background(pcdb, radius=1.65) # 0.9

# RANSAC registration
voxel_size = 0.01 # 0.018 # 0.005 # 0.001 # 0.025 # in meters 
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, pcda, pcdb)
result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

# ICP finetuning
result_icp = refine_registration(source, target, voxel_size, result_ransac.transformation)
cama_to_camb = result_icp.transformation
pcda.transform(cama_to_camb)
o3d.visualization.draw_geometries([pcda, pcdb])

np.save("live_registration/RANSAC/transform_cam" + str(cama) + "_to_cam" + str(camb) + ".npy", cama_to_camb)