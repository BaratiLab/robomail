import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


def generate_camera_extrinsic(cam_translation, 
                               cam_rotation,
                               pos_y_side=False):
    '''
    This function takes in the translation and rotation of a camera and computes the transformation matrix.
    It is handling all the complexities of coordinate frame mismatch between the camera and the robot.

    :param: cam_translation (np.array): 3x1 translation vector for the camera in the robot frame (in meters) [x, y, z]
    :param: cam_rotation (np.array): 3x3 rotation matrix for the camera in the robot frame
    :param: pos_y_side (bool): flag to indicate if the camera is on the positive y side of the robot table (changes how
                               the relative rotation of the camera is translated into the robot frame)

    :returns: cam_to_world (np.array): 4x4 transformation matrix for the camera
    '''
    translation = np.array([cam_translation[0], cam_translation[2], cam_translation[1]])

    # need to change signs and add 180 degrees to the z rotation to match the robot frame
    if pos_y_side:
        if cam_rotation[2] > 0:
            cam_rotation[2] = -(cam_rotation[2] + 180)
        else:
            cam_rotation[2]= -(cam_rotation[2] - 180)
    rotation = Rotation.from_euler('zxy', np.array([cam_rotation[1], cam_rotation[0], -cam_rotation[2]]), degrees=True).as_matrix()

    # create the camera transformation matrix
    cam_transform = np.identity(4)
    cam_transform[:3, :3] = rotation
    cam_transform[:3, 3] = translation

    # adjust the z for calibration offset error
    y_shift_trans = np.identity(4)
    y_shift_trans[1, 3] = -0.8

    # swap the y and z axes of the point cloud with a transformation matrix
    trans_swap_yz = np.identity(4)
    trans_swap_yz[1, 1] = 0
    trans_swap_yz[1, 2] = 1
    trans_swap_yz[2, 1] = 1
    trans_swap_yz[2, 2] = 0

    # create a transformation matrix that flips the sign of the z-axis
    trans_flip_zsign = np.identity(4)
    trans_flip_zsign[2, 2] = -1

    cam_to_world = trans_flip_zsign @ trans_swap_yz @ y_shift_trans @ cam_transform
    return cam_to_world 

def generate_all_camera_transforms(cam2_translation,
                                   cam2_rotation,
                                   cam3_translation,
                                   cam3_rotation,
                                   cam4_translation,
                                   cam4_rotation,
                                   cam5_translation,
                                   cam5_rotation,
                                   correct_offsets=False):
    '''
    This script takes in manual measurements for the camera setup from the camera cage and computes the global transformation 
    matrix to transform points in each camera frame to the robot world coordinate frame. NOTE: This function requires detailed 
    inputs for each camera to be applicable for camera setups where the cameras are not necessarily aligned on certain axes.

    :param: cam2_translation (np.array): 3x1 translation vector for camera 2 in the robot frame (in meters) [x, y, z]
    :param: cam2_rotation (np.array): 3x1 rotation vector for camera 2 in the robot frame (in degrees) [rx, ry, rz]
    :param: cam3_translation (np.array): 3x1 translation vector for camera 3 in the robot frame (in meters) [x, y, z]
    :param: cam3_rotation (np.array): 3x1 rotation vector for camera 3 in the robot frame (in degrees) [rx, ry, rz]
    :param: cam4_translation (np.array): 3x1 translation vector for camera 4 in the robot frame (in meters) [x, y, z]
    :param: cam4_rotation (np.array): 3x1 rotation vector for camera 4 in the robot frame (in degrees) [rx, ry, rz]
    :param: cam5_translation (np.array): 3x1 translation vector for camera 5 in the robot frame (in meters) [x, y, z]
    :param: cam5_rotation (np.array): 3x1 rotation vector for camera 5 in the robot frame (in degrees) [rx, ry, rz]
    :param: correct_offsets (bool): flag to correct for minor offsets in the measurements [versus just going straight from measurements]

    :returns: cam2_transform, cam3_transform, cam4_transform, cam5_transform (np.array): 4x4 transformation matrix for each camera
    '''
    if correct_offsets:
        cam3_translation += np.array([0.0, -0.0395, 0.017])
        cam4_translation += np.array([0.07, -0.0615, 0.0])
        cam5_translation += np.array([0.05, 0.0, 0.027])

    cam2_transform = generate_camera_extrinsic(cam2_translation, cam2_rotation, pos_y_side=False)
    cam3_transform = generate_camera_extrinsic(cam3_translation, cam3_rotation, pos_y_side=False)
    cam4_transform = generate_camera_extrinsic(cam4_translation, cam4_rotation, pos_y_side=True)
    cam5_transform = generate_camera_extrinsic(cam5_translation, cam5_rotation, pos_y_side=True)

    return cam2_transform, cam3_transform, cam4_transform, cam5_transform

# main loop
if __name__ == "__main__":
    # define camera position and rotations from measurements NOTE: these are in meters and degrees!
    cam2_translation = np.array([0.22,-0.4175, 0.45])
    cam2_rotation_euler = np.array([-40.0, 0.0, -30.0])

    cam3_translation = np.array([0.78, -0.4175, 0.45])
    cam3_rotation_euler = np.array([-40.0, 0.0, 30.0])

    cam4_translation = np.array([0.78, 0.4175, 0.45])
    cam4_rotation_euler = np.array([-40.0, 0.0, 30.0])

    cam5_translation = np.array([0.22, 0.4175, 0.45])
    cam5_rotation_euler = np.array([-40.0, 0.0, -30.0])

    cam2_transform, cam3_transform, cam4_transform, cam5_transform = generate_all_camera_transforms(cam2_translation,
                                                                                                    cam2_rotation_euler,
                                                                                                    cam3_translation,
                                                                                                    cam3_rotation_euler,
                                                                                                    cam4_translation,
                                                                                                    cam4_rotation_euler,
                                                                                                    cam5_translation,
                                                                                                    cam5_rotation_euler,
                                                                                                    correct_offsets=True)
    
    # verify the transformation matrices by visualizing scene
    pcl_path = 'Calibration_Data/raw_pcls/'
    pcl2 = o3d.io.read_point_cloud(pcl_path + 'cam2.ply')
    pcl3 = o3d.io.read_point_cloud(pcl_path + 'cam3.ply')
    pcl4 = o3d.io.read_point_cloud(pcl_path + 'cam4.ply')
    pcl5 = o3d.io.read_point_cloud(pcl_path + 'cam5.ply')

    pcl2.transform(cam2_transform)
    pcl3.transform(cam3_transform)
    pcl4.transform(cam4_transform)
    pcl5.transform(cam5_transform)

    # visualize the point clouds after transformation
    o3d.visualization.draw_geometries([pcl2, pcl3, pcl4, pcl5])

    # save the new camera transforms --> TODO: need to manyually move them to /calib in robomail/vision if you are happy with
    # the callibration quality [and create a new file for correct date range to store previous calibration for documentation]
    np.save("registration/realsense_camera2w.npy", cam2_transform)
    np.save("registration/realsense_camera3w.npy", cam3_transform)
    np.save("registration/realsense_camera4w.npy", cam4_transform)
    np.save("registration/realsense_camera5w.npy", cam5_transform)