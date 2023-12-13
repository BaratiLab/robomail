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

WORKFLOW:
(1) Create a new folder in /live_registration for the date you are recalibrating the cameras
(2) Setup the scene as described above
(3) Select recommended starting parameters for clibrating each camera pair.
(4) Run the camera registration script and review the quality of the alignment. RANSAC and ICP use feature matching that
    has some stochasticity, so the quality of alignment can vary greatly. If you see that the system is overfitting the a 
    plane, we'd recommend increasing the voxel size, or experimenting with the options of background removal before both
    or either RANSAC and ICP. If you see that the system is missing finer details, we'd recommend reducing the voxel size.
    If the system is failing with rotational alignment, we recommend using a poster board as a plane in the background to
    improve rotational alignment and ensure that you turn background remove off at least for the RANSAC initial alignment.
(5) Save the calibration extrinsic matrices with different names until you have a camera alignment that you are happy with.
    Please note that the matrices stored in this automatic save folder are not the official extrinsic matrices accessed by 
    the robomail package.
(6) Navigate to /robomail/vision/calib/Past_Calibrations and create a new folder with the current date in the format 
    PreDayMonthYear indicating the transforms that were used pre the current day. Move the current extrinsic matrices to 
    this folder so we have a record of the date periods different extrinsics were used. 
(7) Move the best calibration extrinsics with each camera to /robomail/vision/calib and rename each file to 'realsense_cameraAB'
    where A is the camera frame we are transforming and B is the camera frame we are transforming to. 

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


def find_center_in_cam_frame(cam):
    """
    Some versions of calibration strategy remove the background of the point cloud. This requires
    knoweldge of the center point of the area you intend to keep in the point cloud. Because these
    are uncalibrated cameras, the quick process we use is as follows:
        (1) Place some green clay (or a green object) at the center of the area to preserve.
        (2) Run this script and use the returned center and for the remove_background function.
    """
    # initialize the cameras
    camera = vis.CameraClass(cam)

    # initialize the 3D vision code
    pcl_vis = vis.Vision3D()

    # get the point clouds
    _, _, pc, _ = camera.get_next_frame()
    # pc = pcl_vis.remove_background(pc, radius=1.15) # optional for camera 5 if you're wearing a green shirt
    o3d.visualization.draw_geometries([pc])

    # get the centers in the camera frames
    decolored = pcl_vis.lab_color_crop(pc)
    points = np.asarray(decolored.points)
    center = np.mean(points, axis = 0)

    # visualize what the cloud cropped about the center looks like
    pcd = pcl_vis.remove_background(pc, radius=0.15, center=center)
    o3d.visualization.draw_geometries([pcd])

    # save the centers as numpy arrays
    np.save("cam_centers/cam" + str(cam) + "_center.npy", center)

    return center


def align_two_cameras(cama, camb, voxel_size=0.01, distance_threshold=0.001):
    """
    """
    calib = vis.CalibrationClass()

    # initialize the cameras
    camera_a = vis.CameraClass(cama)
    camera_b = vis.CameraClass(camb)

    # initialize the 3D vision code
    pcl_vis = vis.Vision3D()

    # get the point clouds
    _, _, pca, _ = camera_a.get_next_frame()
    _, _, pcb, _ = camera_b.get_next_frame()

    # combine into single pointcloud
    pcda = o3d.geometry.PointCloud()
    pcda.points = pca.points
    pcda.colors = pca.colors

    pcdb = o3d.geometry.PointCloud()
    pcdb.points = pcb.points
    pcdb.colors = pcb.colors

    # collect addition point clouds to combine
    for i in range(9):
        _, _, pca, _ = camera_a.get_next_frame()
        _, _, pcb, _ = camera_b.get_next_frame()
        
        # combine into single pointcloud
        pcda.points.extend(pca.points)
        pcda.colors.extend(pca.colors)
        pcdb.points.extend(pcb.points)
        pcdb.colors.extend(pcb.colors)

    pcda, _ = pcda.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcdb, _ = pcdb.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    # pcda2 = pcda.copy()
    # pcdb2 = pcdb.copy()

    # load in source center and target center from numpy arrays
    source_center = np.load("cam_centers/cam" + str(cama) + "_center.npy")
    target_center = np.load("cam_centers/cam" + str(camb) + "_center.npy")

    # Remove far background
    pcda = pcl_vis.remove_background(pcda, radius=0.95, center=source_center)
    pcdb = pcl_vis.remove_background(pcdb, radius=0.95, center=target_center)

    # first RANSAC registration with background plane for coarse alignment
    source, target, source_down, target_down, source_fpfh, target_fpfh = calib.prepare_dataset(voxel_size, pcda, pcdb)
    first_result_ransac = calib.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

    # reprocess data with result_ransac transformation and remove background and reduce voxel size
    pcda = pcl_vis.remove_background(pcda, radius=0.15, center=source_center)
    pcdb = pcl_vis.remove_background(pcdb, radius=0.15, center=target_center)
    source, target, source_down, target_down, source_fpfh, target_fpfh = calib.prepare_dataset(voxel_size*0.2, 
                                                                                               pcda.transform(first_result_ransac.transformation), 
                                                                                               pcdb)

    # # Removing everything except calibration object and stage
    # source = pcl_vis.remove_background(pcda2, radius=0.15, center=source_center)
    # target = pcl_vis.remove_background(pcdb2, radius=0.15, center=target_center)

    # second RANSAC registration
    second_result_ransac = calib.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

    # ICP finetuning
    result_ransac = second_result_ransac.transformation # @ first_result_ransac.transformation 
    result_icp = calib.refine_registration(source, target, distance_threshold, result_ransac)
    cama_to_camb = result_icp.transformation
    pcda.transform(cama_to_camb)
    o3d.visualization.draw_geometries([pcda, pcdb])

    np.save("live_registration/13Dec2023/transform_cam" + str(cama) + "_to_cam" + str(camb) + ".npy", cama_to_camb)

if __name__ == "__main__":
    cama = 2
    camb = 3

    # if there are no arrays for camera numbers in /cam_centers folder
    # _ = find_center_in_cam_frame(5)

    # calibrate the two cameras
    align_two_cameras(cama, camb, voxel_size=0.015, distance_threshold=0.001)