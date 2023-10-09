import argparse
import numpy as np
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics

class GetObjectCenterPointInWorld():
    def __init__(self, ee_cam):
        self.ee_cam = ee_cam

    def get_object_center_point_in_world_realsense_3D_camera_point(self, object_camera_point, transform, current_pose):
        """
        """
        object_camera_point = Point(object_camera_point, "realsense_ee")

        # ee mounted camera depends on robot pose
        if self.ee_cam: 
            object_center_point_in_world = current_pose * transform * object_camera_point
        # static camera does not depend on the robot pose
        else:
            object_center_point_in_world = transform * object_camera_point 
        return object_center_point_in_world

class GetPixelPose():
    def __init__(self, cam_serial):
        self.cam_serial = cam_serial

        # dictionary of camera serials
        camera_serials = {
            '220222066259': 1,
            '151322066099': 2,
            '151322069488': 3,
            '151322061880': 4,
            '151322066932': 5,
        }

        # dictionary of camera intrinsics
        camera_intrinsics = {
            1: "calib/realsense_intrinsics.intr",
            2: "calib/realsense_intrinsics_camera2.intr",
            3: "calib/realsense_intrinsics_camera2.intr",
            4: "calib/realsense_intrinsics_camera4.intr",
            5: "calib/realsense_intrinsics_camera5.intr"
        }

        # dictionary of camera extrinsics
        camera_extrinsics = {
            1: "calib/realsense_ee_shifted.tf",
            2: "calib/realsense_camera2.tf",
            3: "calib/realsense_camera3.tf",
            4: "calib/realsense_camera4.tf",
            5: "calib/realsense_camera5.tf"
        }

        # import camera intrinsics and extrinsics
        REALSENSE_INTRINSICS = camera_intrinsics[camera_serials[self.cam_serial]]
        REALSENSE_EE_TF = camera_extrinsics[camera_serials[self.cam_serial]]
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
        )
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_EE_TF)
        args = parser.parse_args()

        self.realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
        self.realsense_to_ee_transform = RigidTransform.load(args.extrinsics_file_path)

        # initialize the base class to calculate the pose
        if camera_serials[self.cam_serial] == 1:
            self.ee_cam = True
        else: 
            self.ee_cam = False

        self.pose_func = GetObjectCenterPointInWorld(self.ee_cam)

    def get_pixel_pose(self, i, j, verts, current_pose):
        """
        INSERT
        """
        obj_points = verts[i, j].reshape(-1,3)
        x = obj_points[:,0] 
        y = obj_points[:,1] 
        z = obj_points[:,2]

        com = self.pose_func.get_object_center_point_in_world_realsense_3D_camera_point(np.array([x,y,z]), self.realsense_to_ee_transform, current_pose)
        com = np.array([com[0], com[1], com[2]])
        return com

class GetPatchPose():
    def __init__(self, cam_serial):
        self.cam_serial = cam_serial

        # dictionary of camera serials
        camera_serials = {
            '220222066259': 1,
            '151322066099': 2,
            '151322069488': 3,
            '151322061880': 4,
            '151322066932': 5,
        }

        # dictionary of camera intrinsics
        camera_intrinsics = {
            1: "calib/realsense_intrinsics.intr",
            2: "calib/realsense_intrinsics_camera2.intr",
            3: "calib/realsense_intrinsics_camera2.intr",
            4: "calib/realsense_intrinsics_camera4.intr",
            5: "calib/realsense_intrinsics_camera5.intr"
        }

        # dictionary of camera extrinsics
        camera_extrinsics = {
            1: "calib/realsense_ee_shifted.tf",
            2: "calib/realsense_camera2.tf",
            3: "calib/realsense_camera3.tf",
            4: "calib/realsense_camera4.tf",
            5: "calib/realsense_camera5.tf"
        }

        # import camera intrinsics and extrinsics
        REALSENSE_INTRINSICS = camera_intrinsics[camera_serials[self.cam_serial]]
        REALSENSE_EE_TF = camera_extrinsics[camera_serials[self.cam_serial]]
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
        )
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_EE_TF)
        args = parser.parse_args()

        self.realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
        self.realsense_to_ee_transform = RigidTransform.load(args.extrinsics_file_path)

        # initialize the base class to calculate the pose
        if camera_serials[self.cam_serial] == 1:
            self.ee_cam = True
        else: 
            self.ee_cam = False

        self.pose_func = GetObjectCenterPointInWorld(self.ee_cam)


    def get_patch_pose(self, bounds, verts, tag_translation, current_pose):
        """
        """
        minx = np.amin(bounds[:,0], axis=0)
        maxx = np.amax(bounds[:,0], axis=0)
        miny = np.amin(bounds[:,1], axis=0)
        maxy = np.amax(bounds[:,1], axis=0)
        
        obj_points = verts[miny:maxy, minx:maxx].reshape(-1,3)
        
        zs = obj_points[:,2]
        z = np.median(zs)
        xs = obj_points[:,0]
        ys = obj_points[:,1]
        ys = np.delete(ys, np.where((zs < z - 1) | (zs > z + 1))) # take only y for close z to prevent including background
        
        x_pos = np.median(xs)
        y_pos = np.median(ys)
        z_pos = z
        
        median_point = np.array([x_pos, y_pos, z_pos])
        
        object_median_point = self.pose_func.get_object_center_point_in_world_realsense_3D_camera_point(median_point, self.realsense_to_ee_transform, current_pose)
        com_depth = np.array([object_median_point[0], object_median_point[1], object_median_point[2]])
        
        # ---- Image-Based Prediction (No Depth) ----
        com_nodepth = self.pose_func.get_object_center_point_in_world_realsense_3D_camera_point(tag_translation, self.realsense_to_ee_transform, current_pose)
        com_nodepth = np.array([com_nodepth[0], com_nodepth[1], com_nodepth[2]])
        
        # ---- Combine Predictions ----
        # if depth-based prediction is Nan, only use non-depth-based prediction
        if np.isnan(com_depth.any()):
            com_depth = com_nodepth
        
        # if the prediction difference between depth and no depth is large ignore depth-based z
        elif abs(com_depth[2] - com_nodepth[2]) > 0.05:
            com_depth = com_nodepth
            
        # weighted average
        object_center_point = np.array([(com_depth[0] + com_nodepth[0])/2, (com_depth[1] + com_nodepth[1])/2, (2*com_depth[2] + com_nodepth[2])/3])
        return object_center_point
    
    def get_intrinsics_extrinsics(self):
        return self.realsense_intrinsics, self.realsense_to_ee_transform