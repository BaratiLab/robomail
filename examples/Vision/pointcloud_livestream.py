import time
import numpy as np
import open3d as o3d
import robomail.vision as vis

# initialize the cameras
cam2 = vis.CameraClass(2)
cam3 = vis.CameraClass(3)
cam4 = vis.CameraClass(4)
cam5 = vis.CameraClass(5)

# get the camera to world transforms
transform_2w = cam2._get_cam_extrinsics()
transform_3w = cam3._get_cam_extrinsics()
transform_4w = cam4._get_cam_extrinsics()
transform_5w = cam5._get_cam_extrinsics()

# initialize the 3D vision code
pcl_vis = vis.Vision3D()

# Create visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()

_, _, pc2, _ = cam2._get_next_frame()
_, _, pc3, _ = cam3._get_next_frame()
_, _, pc4, _ = cam4._get_next_frame()
_, _, pc5, _ = cam5._get_next_frame()

# transform cameras
pc2.transform(transform_2w)
pc3.transform(transform_3w)
pc4.transform(transform_4w)
pc5.transform(transform_5w)

# combine into single pointcloud
pointcloud = o3d.geometry.PointCloud()
pointcloud.points = pc3.points
pointcloud.colors = pc3.colors
pointcloud.points.extend(pc2.points)
pointcloud.colors.extend(pc2.colors)
pointcloud.points.extend(pc4.points)
pointcloud.colors.extend(pc4.colors)
pointcloud.points.extend(pc5.points)
pointcloud.colors.extend(pc5.colors)

pointcloud = pcl_vis.remove_background(pointcloud, radius=1.15) 

vis.add_geometry(pointcloud)
vis.poll_events()
vis.update_renderer()

while True:
    _, _, pc2, _ = cam2._get_next_frame()
    _, _, pc3, _ = cam3._get_next_frame()
    _, _, pc4, _ = cam4._get_next_frame()
    _, _, pc5, _ = cam5._get_next_frame()

    # transform cameras
    pc2.transform(transform_2w)
    pc3.transform(transform_3w)
    pc4.transform(transform_4w)
    pc5.transform(transform_5w)

    # combine into single pointcloud
    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = pc3.points
    pointcloud.colors = pc3.colors
    pointcloud.points.extend(pc2.points)
    pointcloud.colors.extend(pc2.colors)
    pointcloud.points.extend(pc4.points)
    pointcloud.colors.extend(pc4.colors)
    pointcloud.points.extend(pc5.points)
    pointcloud.colors.extend(pc5.colors)

    pointcloud = pcl_vis.remove_background(pointcloud, radius=1.15) 

    vis.add_geometry(pointcloud)
    vis.poll_events()
    vis.update_renderer()