import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

'''
This script takes in manual measurements for the camera setup from the camera cage and computes the transformation matrix.
To test these extrinsics, we import pre-recorded point clouds from the cameras and apply the transformation to align them.
'''

# manually define the translation and rotation of each camera from measurements/CAD model of camera cage
# cam2_translation = np.array([22,-41.75, 45])
# cam2_rotation_euler = np.array([-40.0, 0.0, -30.0])
cam2_translation = np.array([0.22, 0.45, -0.4175])
cam2_rotation = Rotation.from_euler('zxy', np.array([0.0, -40.0, 30.0]), degrees=True).as_matrix() # when flipped 0, 40, -30 worked

# cam3_translation = np.array([78, -41.75, 45])
# cam3_rotation_euler = np.array([-40.0, 0.0, 30.0])
cam3_translation = np.array([0.78, 0.467, -0.457]) # axes are x, z, y
cam3_rotation = Rotation.from_euler('zxy', np.array([0.0, -40.0, -30.0]), degrees=True).as_matrix() # when flipped 0, 40, 30 worked

# cam4_translation = np.array([78, 41.75, 45])
# cam4_rotation_euler = np.array([40.0, 0.0, 30.0])
# cam4_translation = np.array([-41.75, -45, -78])
cam4_translation = np.array([0.85, 0.45, 0.366])
cam4_rotation = Rotation.from_euler('zxy', np.array([0.0, -40.0, 180+30.0]), degrees=True).as_matrix()
print("cam4 translation: ", cam4_translation)

# cam5_translation = np.array([22, 41.75, 45])
# cam5_rotation_euler = np.array([40.0, 0.0, -30.0])
# cam5_translation = np.array([-41.75, -45, -22])
cam5_translation = np.array([0.27, 0.45, 0.3905])
cam5_rotation = Rotation.from_euler('zxy', np.array([0.0, -40.0, -(180+30.0)]), degrees=True).as_matrix()

# RELATIVE CAM4 - CAM5 DIFFERENCES
# [0.58, 0, -0.0345]

# import the point clouds from the cameras
pcl_path = '/home/alison/pointclouds/'
cam2_pcl = o3d.io.read_point_cloud(pcl_path + 'pc2.ply')
cam3_pcl = o3d.io.read_point_cloud(pcl_path + 'pc3.ply')
cam4_pcl = o3d.io.read_point_cloud(pcl_path + 'pc4.ply')
cam5_pcl = o3d.io.read_point_cloud(pcl_path + 'pc5.ply')

# visualize the point clouds before transformation
# o3d.visualization.draw_geometries([cam2_pcl, cam3_pcl, cam4_pcl, cam5_pcl])

# create the transformation matrices for each camera
# cam2_rotation = o3d.geometry.get_rotation_matrix_from_xyz(cam2_rotation_euler)
cam2_transform = np.identity(4)
cam2_transform[:3, :3] = cam2_rotation
cam2_transform[:3, 3] = cam2_translation

# cam3_rotation = o3d.geometry.get_rotation_matrix_from_xyz(cam3_rotation_euler)
cam3_transform = np.identity(4)
cam3_transform[:3, :3] = cam3_rotation
cam3_transform[:3, 3] = cam3_translation

# cam4_rotation = o3d.geometry.get_rotation_matrix_from_xyz(cam4_rotation_euler)
cam4_transform = np.identity(4)
cam4_transform[:3, :3] = cam4_rotation
cam4_transform[:3, 3] = cam4_translation

# cam5_rotation = o3d.geometry.get_rotation_matrix_from_xyz(cam5_rotation_euler)
cam5_transform = np.identity(4)
cam5_transform[:3, :3] = cam5_rotation
cam5_transform[:3, 3] = cam5_translation

# apply transformation to the point clouds
cam2_pcl.transform(cam2_transform)
cam3_pcl.transform(cam3_transform)
cam4_pcl.transform(cam4_transform)
cam5_pcl.transform(cam5_transform)

# visualize the point clouds after transformation
# o3d.visualization.draw_geometries([cam2_pcl, cam3_pcl, cam4_pcl, cam5_pcl])

# get the mean values of all camera point clouds
cam2_pcl_points = np.asarray(cam2_pcl.points)
cam3_pcl_points = np.asarray(cam3_pcl.points)
cam4_pcl_points = np.asarray(cam4_pcl.points)
cam5_pcl_points = np.asarray(cam5_pcl.points)
cam2_pcl_mean = np.mean(cam2_pcl_points, axis=0)
cam3_pcl_mean = np.mean(cam3_pcl_points, axis=0)
cam4_pcl_mean = np.mean(cam4_pcl_points, axis=0)
cam5_pcl_mean = np.mean(cam5_pcl_points, axis=0)
print("\nMean cam2: ", cam2_pcl_mean)
print("Mean cam3: ", cam3_pcl_mean)
print("Mean cam4: ", cam4_pcl_mean)
print("Mean cam5: ", cam5_pcl_mean)

# create a combined point cloud with all the camera points
pointcloud = o3d.geometry.PointCloud()
pointcloud.points = cam2_pcl.points
pointcloud.points.extend(cam3_pcl.points)
pointcloud.points.extend(cam4_pcl.points)
pointcloud.points.extend(cam5_pcl.points)
# o3d.visualization.draw_geometries([pointcloud])

# remove the background from the combined point cloud
full_points = np.asarray(pointcloud.points)
full_points = full_points[full_points[:, 0] > 0.0]
full_points = full_points[full_points[:, 1] > 0.8]
full_points = full_points[full_points[:, 2] > -0.2]
full_points = full_points[full_points[:, 2] < 0.2]

# adjust the z for calibration offset error
y_shift_trans = np.identity(4)
y_shift_trans[1, 3] = -0.8
cropped_pointcloud = o3d.geometry.PointCloud()
cropped_pointcloud.points = o3d.utility.Vector3dVector(full_points)
cropped_pointcloud.transform(y_shift_trans)
# o3d.visualization.draw_geometries([cropped_pointcloud])

# get the min/max/mean of the cropped point cloud
min = np.min(full_points, axis=0)
max = np.max(full_points, axis=0)
mean = np.mean(full_points, axis=0)
print("\nMin: ", min)
print("Max: ", max)
print("Mean: ", mean)

# swap the y and z axes of the point cloud with a transformation matrix
trans_swap_yz = np.identity(4)
trans_swap_yz[1, 1] = 0
trans_swap_yz[1, 2] = 1
trans_swap_yz[2, 1] = 1
trans_swap_yz[2, 2] = 0
# cropped_pointcloud.transform(trans_swap_yz)

# create a transformation matrix that flips the sign of the z-axis
trans_flip_zsign = np.identity(4)
trans_flip_zsign[2, 2] = -1
cropped_pointcloud.transform(trans_flip_zsign)
# o3d.visualization.draw_geometries([cropped_pointcloud])
new_points = np.asarray(cropped_pointcloud.points)
min = np.min(new_points, axis=0)
max = np.max(new_points, axis=0)
mean = np.mean(new_points, axis=0)
print("\nMin: ", min)
print("Max: ", max)
print("Mean: ", mean)

# re-import the point clouds and apply the tranformations to verify the global transforms are correct
global_cam2_transform = trans_flip_zsign @ trans_swap_yz @ y_shift_trans @ cam2_transform 
global_cam3_transform = trans_flip_zsign @ trans_swap_yz @ y_shift_trans @ cam3_transform 
global_cam4_transform = trans_flip_zsign @ trans_swap_yz @ y_shift_trans @ cam4_transform 
global_cam5_transform = trans_flip_zsign @ trans_swap_yz @ y_shift_trans @ cam5_transform 

pcl2 = o3d.io.read_point_cloud(pcl_path + 'pc2.ply')
pcl3 = o3d.io.read_point_cloud(pcl_path + 'pc3.ply')
pcl4 = o3d.io.read_point_cloud(pcl_path + 'pc4.ply')
pcl5 = o3d.io.read_point_cloud(pcl_path + 'pc5.ply')

pcl2.transform(global_cam2_transform)
pcl3.transform(global_cam3_transform)
pcl4.transform(global_cam4_transform)
pcl5.transform(global_cam5_transform)

# visualize the point clouds after transformation
o3d.visualization.draw_geometries([pcl2, pcl3, pcl4, pcl5])
# o3d.visualization.draw_geometries([pcl2, pcl4])

# print out final transforms:
print("\nCam2: ", global_cam2_transform)
print("\nCam3: ", global_cam3_transform)
print("\nCam4: ", global_cam4_transform)
print("\nCam5: ", global_cam5_transform)