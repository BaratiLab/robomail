import time
import numpy as np
import open3d as o3d
import robomail.vision as vis

# initialize the cameras
cam2 = vis.CameraClass(2)
cam3 = vis.CameraClass(3)
cam4 = vis.CameraClass(4)
cam5 = vis.CameraClass(5)

# initialize the 3D vision code
pcl_vis = vis.Vision3D()

# get the point clouds
_, _, pc2, _ = cam2.get_next_frame()
_, _, pc3, _ = cam3.get_next_frame()
_, _, pc4, _ = cam4.get_next_frame()
_, _, pc5, _ = cam5.get_next_frame()

scaled_pcl = pcl_vis.fuse_point_clouds(pc2, pc3, pc4, pc5)

# visualize the returned scaled point cloud
pointcloud = o3d.geometry.PointCloud()
pointcloud.points = o3d.utility.Vector3dVector(scaled_pcl)
pointcloud.colors = o3d.utility.Vector3dVector(np.tile(np.array([0,0,1]), (len(scaled_pcl),1)))