import open3d as o3d
import copy
import time
import numpy as np
import numpy.typing as npt
from .cam_utils import get_cam_info
from skimage.color import rgb2lab
from matplotlib import pyplot as plt
from shapely.geometry import Point, Polygon


class Vision3D:
    def __init__(self, transform_list=[]):
        # save a dictionary of the camera transforms (should be from each static camera to world frame)
        self.camera_transforms = {}
        if len(transform_list) > 0:
            for i, transform in enumerate(transform_list):
                self.camera_transforms[i + 2] = transform
        else:
            for i in range(1, 6):
                _, transform, _ = get_cam_info(i)
                self.camera_transforms[i] = transform  # .matrix()
                # _, self.camera_transforms[i], _ = get_cam_info(i)

    def plot_target_and_state_clouds(self, state, target):
        """
        Given two point clouds, plot them using open3d.
        """
        target_pcl = o3d.geometry.PointCloud()
        target_pcl.points = o3d.utility.Vector3dVector(target)
        target_colors = np.tile(np.array([1, 0, 0]), (len(target), 1))
        target_pcl.colors = o3d.utility.Vector3dVector(target_colors)

        pcl = o3d.geometry.PointCloud()
        pcl.points = o3d.utility.Vector3dVector(state)
        pcl_colors = np.tile(np.array([0, 0, 1]), (len(state), 1))
        pcl.colors = o3d.utility.Vector3dVector(pcl_colors)
        return target_pcl, pcl

    def pcl_to_image(self, pcd1, pcd2, save_path):
        """
        Saves image on Open3D visualizer as PNG

        Args:
        pcd1 (o3d.geometry.PointCloud): input point cloud to visualize
        pcd2 (Optional[o3d.geometry.PointCloud]): additional input point cloud to visualize
        save_path (string): location to save PNG
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd1)

        if pcd2:
            vis.add_geometry(pcd2)

        vis.run()
        image = vis.capture_screen_float_buffer(True)
        plt.imsave(save_path, np.asarray(image))
        vis.destroy_window()
        del vis

    def remove_stage_grippers(self, pcd):
        """
        Removes points from input point cloud based on a constant Z threshold to remove
        the clay stage and robot grippers

        Args:
        pcd (o3d.geometry.PointCloud): input point cloud

        Returns:
        pcd (o3d.geometry.PointCloud): output point cloud with clay stage and grippers removed
        """
        # removing the elevated stage
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        ind_z_upper = np.where(points[:, 2] > 0.065)# > 0.207)
        pcd.points = o3d.utility.Vector3dVector(points[ind_z_upper])
        pcd.colors = o3d.utility.Vector3dVector(colors[ind_z_upper])

        # removing the grippers
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        ind_z_lower = np.where(points[:, 2] < 0.15)
        pcd.points = o3d.utility.Vector3dVector(points[ind_z_lower])
        pcd.colors = o3d.utility.Vector3dVector(colors[ind_z_lower])

        return pcd

    def remove_background(
        self, pcd, radius: float = 0.9, center: npt.NDArray = np.array([0, 0, 0])
    ):
        """
        Removes points from intput point cloud outside of a sphere of given radius
        and center

        Args:
        pcd (o3d.geometry.PointCloud): input point cloud
        radius (float): radius of sphere outside of which to remove points
        center (np.array): coordinate of center of sphere outside of which to remove points

        Returns:
        pcd (o3d.geometry.PointCloud): output point cloud with points removed
        """
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)

        distances = np.linalg.norm(points - center, axis=1)
        indices = np.where(distances <= radius)

        pcd.points = o3d.utility.Vector3dVector(points[indices])
        pcd.colors = o3d.utility.Vector3dVector(colors[indices])

        return pcd

    def lab_color_crop(self, pcd_incoming, color='Green'):
        """
        Removes points from input point cloud based on color in LAB color space
        (currently thresholds for a green color)

        Args:
        pcd_incoming (o3d.geometry.PointCloud): input point cloud

        Returns:
        pcd (o3d.geometry.PointCloud): output point cloud with points removed
        """

        pcd = copy.deepcopy(pcd_incoming)

        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        lab_colors = rgb2lab(colors)  # covert colors to LAB color space

        if color == 'Green':
            a_idx = np.where(lab_colors[:, 1] < -5)
            l_idx = np.where(lab_colors[:, 0] > 0)
            b_idx = np.where(lab_colors[:, 2] > -5)
        elif color == 'Orange':
            # a_idx = (np.where(lab_colors[:, 1] > -45)) and (np.where(lab_colors[:,1] < 45)) # -5 # 0
            # l_idx = np.where(lab_colors[:, 0] > 0) # 0
            # b_idx = np.where(lab_colors[:, 2] < -5)
            a_idx = (np.where(lab_colors[:, 1] > -45)) and (np.where(lab_colors[:,1] < 80)) # -5 # 0
            l_idx = np.where(lab_colors[:, 0] > 0) # 0
            b_idx = np.where(lab_colors[:, 2] < -5)
        else:
            raise ValueError("Invalid color, needs to be implemented!")

        indices = np.intersect1d(a_idx, np.intersect1d(l_idx, b_idx))

        pcd.points = o3d.utility.Vector3dVector(points[indices])
        pcd.colors = o3d.utility.Vector3dVector(colors[indices])

        return pcd
    
    def no_base_plane_unnormalize_point_cloud_fusal(self, pc2, pc3, pc4, pc5):
        """
        Stitches together 4 point clouds from cameras 2, 3, 4, and 5 in the
        Franka workspace into world coordinate frame, performs background
        and color cropping.

        Args:
        pc2 (o3d.geometry.PointCloud): point cloud from camera 2
        pc3 (o3d.geometry.PointCloud): point cloud from camera 3
        pc4 (o3d.geometry.PointCloud): point cloud from camera 4
        pc5 (o3d.geometry.PointCloud): point cloud from camera 5
        vis (bool): if True, will display final results (default False)

        Returns:
        unscaled, unnormalized cloud (o3d.geometry.PointCloud): stitched cloud no base plane 
        for faster computation
        """
        # transform each cloud to world frame
        pc2.transform(self.camera_transforms[2])
        pc3.transform(self.camera_transforms[3])
        pc4.transform(self.camera_transforms[4])
        pc5.transform(self.camera_transforms[5])

        # combine the point clouds
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = pc5.points
        pointcloud.colors = pc5.colors
        pointcloud.points.extend(pc2.points)
        pointcloud.colors.extend(pc2.colors)
        pointcloud.points.extend(pc3.points)
        pointcloud.colors.extend(pc3.colors)
        pointcloud.points.extend(pc4.points)
        pointcloud.colors.extend(pc4.colors)

        # crop point cloud
        pointcloud, ind = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )  # remove statistical outliers
        pointcloud = self.remove_stage_grippers(pointcloud)
        pointcloud = self.remove_background(
            pointcloud, radius=0.15, center=np.array([0.6, -0.05, 0.3])
        )

        # color thresholding
        pointcloud = self.lab_color_crop(pointcloud)
        pointcloud, ind = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )

        pointcloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([0, 0, 1]), (len(pointcloud.points), 1))
        )

        # uniformly sample 2048 points from each point cloud
        points = np.asarray(pointcloud.points)
        idxs = np.random.randint(0, len(points), size=2048)
        points = points[idxs]

        pc_center = pointcloud.get_center() 
        return points, pc_center

    def get_point_cloud_elevated_stage(self, pc2, pc3, pc4, pc5, no_transformation=False):
        if not no_transformation:
            # transform each cloud to world frame
            pc2.transform(self.camera_transforms[2])
            pc3.transform(self.camera_transforms[3])
            pc4.transform(self.camera_transforms[4])
            pc5.transform(self.camera_transforms[5])

        # combine the point clouds
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = pc5.points
        pointcloud.colors = pc5.colors
        pointcloud.points.extend(pc2.points)
        pointcloud.colors.extend(pc2.colors)
        pointcloud.points.extend(pc3.points)
        pointcloud.colors.extend(pc3.colors)
        pointcloud.points.extend(pc4.points)
        pointcloud.colors.extend(pc4.colors)

        # o3d.visualization.draw_geometries([pointcloud])

        # print("test: ", np.asarray(pointcloud.points).shape)

        # crop point cloud
        pointcloud, ind = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )  # remove statistical outliers
        pointcloud = self.remove_stage_grippers(pointcloud)
        pointcloud = self.remove_background(
            pointcloud, radius=0.15, center=np.array([0.6, -0.05, 0.3])
        )
        return pointcloud

    def get_unprocessed_clay(self, pc2, pc3, pc4, pc5, no_transformation=False, color='Green'):
        """
        Stitches together 4 point clouds from cameras 2, 3, 4, and 5. Then,
        color thresholds based on the clay color and returns only the 
        dense clay point cloud as a numpy array.
        """
        if not no_transformation:
            # transform each cloud to world frame with camera calibration transformation matrices
            pc2.transform(self.camera_transforms[2])
            pc3.transform(self.camera_transforms[3])
            pc4.transform(self.camera_transforms[4])
            pc5.transform(self.camera_transforms[5])

        # combine the point clouds
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = pc5.points
        pointcloud.colors = pc5.colors
        pointcloud.points.extend(pc2.points)
        pointcloud.colors.extend(pc2.colors)
        pointcloud.points.extend(pc3.points)
        pointcloud.colors.extend(pc3.colors)
        pointcloud.points.extend(pc4.points)
        pointcloud.points.extend(pc4.colors) 

        # visualize the point cloud
        o3d.visualization.draw_geometries([pointcloud])

        pointcloud, ind = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )  # remove statistical outliers

        # visualize the point cloud
        o3d.visualization.draw_geometries([pointcloud])


        pointcloud = self.remove_stage_grippers(pointcloud)

        # visualize the point cloud
        o3d.visualization.draw_geometries([pointcloud])

        pointcloud = self.remove_background(
            pointcloud, radius=0.15, center=np.array([0.6, 0.0, 0.15])# np.array([0.6, -0.05, 0.3])
        )

        # visualize the point cloud
        o3d.visualization.draw_geometries([pointcloud])

        # color thresholding
        pointcloud = self.lab_color_crop(pointcloud, color)
        pointcloud, ind = pointcloud.remove_radius_outlier(
            nb_points=20, radius=0.005
        ) 

        # visualize the point cloud
        o3d.visualization.draw_geometries([pointcloud])

        return np.asarray(pointcloud.points)
    
    def get_clay(self, pc2, pc3, pc4, pc5, no_transformation=False, color='Green'):
        """
        Stitches together 4 point clouds from cameras 2, 3, 4, and 5. Then,
        color thresholds based on the clay color and returns only the 
        dense clay point cloud.
        """
        if not no_transformation:
            # transform each cloud to world frame
            pc2.transform(self.camera_transforms[2])
            pc3.transform(self.camera_transforms[3])
            pc4.transform(self.camera_transforms[4])
            pc5.transform(self.camera_transforms[5])

        # combine the point clouds
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = pc5.points
        pointcloud.colors = pc5.colors
        pointcloud.points.extend(pc2.points)
        pointcloud.colors.extend(pc2.colors)
        pointcloud.points.extend(pc3.points)
        pointcloud.colors.extend(pc3.colors)
        pointcloud.points.extend(pc4.points)
        pointcloud.colors.extend(pc4.colors)

        # # bgr --> rgb
        # full_scene = pointcloud
        # color_array = np.asarray(full_scene.colors)
        # print("\nColor Array Shape: ", color_array.shape)
        # flipped_colors = np.concatenate([np.expand_dims(color_array[:,2], axis=1), np.expand_dims(color_array[:,1],axis=1), np.expand_dims(color_array[:,0], axis=1)], axis=1)
        # print("\nFlipped Color Shape: ", flipped_colors.shape)
        # full_scene.colors = o3d.utility.Vector3dVector(flipped_colors)
        # o3d.visualization.draw_geometries([full_scene]) # TODO: delete after getting the necessary images

        pointcloud, ind = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )  # remove statistical outliers
        pointcloud = self.remove_stage_grippers(pointcloud)
        pointcloud = self.remove_background(
            pointcloud, radius=0.15, center=np.array([0.6, 0.0, 0.15])# np.array([0.6, -0.05, 0.3])
        )
        

        # color thresholding
        pointcloud = self.lab_color_crop(pointcloud, color)
        pointcloud, ind = pointcloud.remove_radius_outlier(
            nb_points=20, radius=0.005
        ) 

        # get shape of clay base
        pointcloud.estimate_normals()
        downpdc = pointcloud.voxel_down_sample(voxel_size=0.0025)
        downpdc_points = np.asarray(downpdc.points)

        polygon_indices = np.where(downpdc_points[:, 2] < 0.075) 
        polygon_pcl = o3d.geometry.PointCloud()
        polygon_pcl.points = o3d.utility.Vector3dVector(downpdc_points[polygon_indices])

        # generate a 2d grid of points for the base
        base_plane = []
        minx, maxx = np.amin(downpdc_points[:, 0]), np.amax(downpdc_points[:, 0])
        miny, maxy = np.amin(downpdc_points[:, 1]), np.amax(downpdc_points[:, 1])
        minz, maxz = np.amin(downpdc_points[:, 2]), np.amax(downpdc_points[:, 2])
        x_vals = np.linspace(minx, maxx, 60)
        y_vals = np.linspace(miny, maxy, 60)
        xx, yy = np.meshgrid(
            x_vals, y_vals
        )  # create grid that covers full area of 2d polygon

        z = 0.0675
        zz = np.ones(len(xx.flatten())) * z
        points = np.vstack((xx.flatten(), yy.flatten(), zz)).T

        grid_cloud = o3d.geometry.PointCloud()
        grid_cloud.points = o3d.utility.Vector3dVector(points)

        # crop shape of clay base out of 2d grid of points
        polygon_coords = np.asarray(polygon_pcl.points)[:, 0:2]
        polygon = Polygon(polygon_coords)
        mask = [
            polygon.contains(Point(x, y))
            for x, y in np.asarray(grid_cloud.points)[:, 0:2]
        ]
        cropped_grid = np.asarray(grid_cloud.points)[:, 0:2][mask]
        zs = np.ones(len(cropped_grid)) * z
        cropped_grid = np.concatenate(
            (cropped_grid, np.expand_dims(zs, axis=1)), axis=1
        )

        base_cloud = o3d.geometry.PointCloud()
        base_cloud.points = o3d.utility.Vector3dVector(cropped_grid)

        # add top part of clay to new base
        base_cloud.points.extend(downpdc.points)
        cropped_plane, ind = base_cloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )

        base_cloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([0, 0, 1]), (len(base_cloud.points), 1))
        )

        points = np.asarray(base_cloud.points)
        pc_center = base_cloud.get_center() 
        return points, pc_center
    
    def crop_point_clouds_separately(self, pc1, pc2, pc3, pc4, pc5, no_transformation=False, color="Orange", ee_pos=np.ones(3), ee_rot=np.zeros((3,3)), icp=False):
        if not no_transformation:
            # get transform matrix from translation and rotation
            pose_transform = np.eye(4)
            pose_transform[:3,:3] = ee_rot
            pose_transform[0:3,3] = ee_pos
            pc1.transform(self.camera_transforms[1]).transform(pose_transform)
            pc1.translate((0.065,-0.005,-0.01)) # NOTE: this is the calibration offset for ee pose: [0.625, 0.000, 0.325] 

            # rotate end-effector camera by 180 degrees
            R = pc1.get_rotation_matrix_from_xyz((0, 0, np.pi))
            pc1.rotate(R, center=pc1.get_center())
            pc1.translate((-0.06,-0.03,0.001))

            # tralate fixed pcls for improved visualization
            pc2.translate((-0.0015, 0.0005, 0.0)) 
            pc3.translate((0.0015, -0.0015, 0.0))
            pc4.translate((0.0, 0.0, -0.005))
            pc5.translate((0.0, 0.004, 0.0))

            # apply the transforms to each camera
            pc2.transform(self.camera_transforms[2])
            pc3.transform(self.camera_transforms[3])
            pc4.transform(self.camera_transforms[4])
            pc5.transform(self.camera_transforms[5])

        # remove statistical outliers
        pc1, ind = pc1.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pc2, ind = pc2.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pc3, ind = pc3.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pc4, ind = pc4.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pc5, ind = pc5.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # remove grippers
        pc1 = self.remove_stage_grippers(pc1)
        pc2 = self.remove_stage_grippers(pc2)
        pc3 = self.remove_stage_grippers(pc3)
        pc4 = self.remove_stage_grippers(pc4)
        pc5 = self.remove_stage_grippers(pc5)

        # remove background
        pc1 = self.remove_background(pc1, radius=0.15, center=np.array([0.6, 0.0, 0.15]))
        pc2 = self.remove_background(pc2, radius=0.15, center=np.array([0.6, 0.0, 0.15]))
        pc3 = self.remove_background(pc3, radius=0.15, center=np.array([0.6, 0.0, 0.15]))
        pc4 = self.remove_background(pc4, radius=0.15, center=np.array([0.6, 0.0, 0.15]))
        pc5 = self.remove_background(pc5, radius=0.15, center=np.array([0.6, 0.0, 0.15]))

        # color thresholding
        pc1 = self.lab_color_crop(pc1, color)
        pc2 = self.lab_color_crop(pc2, color)
        pc3 = self.lab_color_crop(pc3, color)
        pc4 = self.lab_color_crop(pc4, color)
        pc5 = self.lab_color_crop(pc5, color)

        # remove outliers again
        pc1, ind = pc1.remove_radius_outlier(nb_points=20, radius=0.005) 
        pc2, ind = pc2.remove_radius_outlier(nb_points=20, radius=0.005) 
        pc3, ind = pc3.remove_radius_outlier(nb_points=20, radius=0.005) 
        pc4, ind = pc4.remove_radius_outlier(nb_points=20, radius=0.005) 
        pc5, ind = pc5.remove_radius_outlier(nb_points=20, radius=0.005) 

        # estimate normals
        pc1.estimate_normals()
        pc2.estimate_normals()
        pc3.estimate_normals()
        pc4.estimate_normals()
        pc5.estimate_normals()

        # get downsampled 
        down_pc1 = pc1.voxel_down_sample(voxel_size=0.0025)
        down_pc2 = pc2.voxel_down_sample(voxel_size=0.0025)
        down_pc3 = pc3.voxel_down_sample(voxel_size=0.0025)
        down_pc4 = pc4.voxel_down_sample(voxel_size=0.0025)
        down_pc5 = pc5.voxel_down_sample(voxel_size=0.0025)

        if icp == False:
            return down_pc1, down_pc2, down_pc3, down_pc4, down_pc5
        
        else:
            # try icp alignment with substantially downsampled point cloud
            target = o3d.geometry.PointCloud()
            target.points = down_pc2.points
            target.colors = down_pc2.colors
            target.points.extend(down_pc3.points)
            target.colors.extend(down_pc3.colors)
            target.points.extend(down_pc4.points)
            target.colors.extend(down_pc4.colors)
            target.points.extend(down_pc5.points)
            target.colors.extend(down_pc5.colors)

            # downsample target to fixed 1000 points
            target_downsampled = copy.deepcopy(target)
            target_downsampled = target.voxel_down_sample(voxel_size=0.005)
            source = copy.deepcopy(down_pc1)
            source = down_pc1.voxel_down_sample(voxel_size=0.005)
            # o3d.visualization.draw_geometries([source, target])

            # get icp aligmnet
            threshold = 0.01 #1 # 07 # 05
            trans_init = np.asarray([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0], [0.0, 0.0, 0.0, 1.0]])
        
            reg_p2p = o3d.pipelines.registration.registration_icp(source, target_downsampled, threshold, trans_init, 
                                                                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2500))
            source_copy = copy.deepcopy(down_pc1)
            source_copy.transform(reg_p2p.transformation)

            # add a final combination of the point clouds and remove outliers one more time after combined
            calibrated_pcl = o3d.geometry.PointCloud()
            calibrated_pcl.points = target.points
            calibrated_pcl.colors = target.colors
            calibrated_pcl.points.extend(source_copy.points)
            calibrated_pcl.colors.extend(source_copy.colors)

            calibrated_pcl, ind = calibrated_pcl.remove_statistical_outlier(nb_neighbors=25, std_ratio=2.0)

            # downsample the accepted point cloud to fixed 2048 points
            points = np.asarray(calibrated_pcl.points)
            idxs = np.random.randint(0, len(points), size=2048)
            # TODO: change to have certain number of points removed per pov (need to have some points from ee)
            downsampled_points = points[idxs]
            print("\n\nDownsampled points shape: ", downsampled_points.shape)

            # get the center
            pc_center = calibrated_pcl.get_center() 

            return down_pc1, down_pc2, down_pc3, down_pc4, down_pc5, downsampled_points, pc_center

        # # uniformly sample 2048 points from each point cloud
        # points = np.asarray(downpdc.points)
        # idxs = np.random.randint(0, len(points), size=2048)
        # points = points[idxs]

        # # re-process the processed_pcl to center
        # # pc_center = np.array([0.6, 0.0, 0.24])
        # pc_center = downpdc.get_center() 
        # return points, pc_center
    
    def unnormalize_fuse_point_clouds_no_base_ee_cam(self, pc1, pc2, pc3, pc4, pc5, no_transformation=False, color="Orange", ee_pos=np.ones(3), ee_rot=np.zeros((3,3))):
        if not no_transformation:
            # get transform matrix from translation and rotation
            pose_transform = np.eye(4)
            pose_transform[:3,:3] = ee_rot
            pose_transform[0:3,3] = ee_pos
            # transform each cloud to world frame
            # pc1.transform(pose_transform).transform(self.camera_transforms[1])
            pc1.transform(self.camera_transforms[1]).transform(pose_transform)
            # handle calibration offsets
            # pc1.translate((-0.05,-0.025,-0.01))
            pc1.translate((0.065,-0.005,-0.01)) # NOTE: this is the calibration offset for ee pose: [0.625, 0.000, 0.325] 
            pc2.transform(self.camera_transforms[2])
            pc3.transform(self.camera_transforms[3])
            pc4.transform(self.camera_transforms[4])
            pc5.transform(self.camera_transforms[5])

        # combine the point clouds
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = pc5.points
        pointcloud.colors = pc5.colors
        pointcloud.points.extend(pc2.points)
        pointcloud.colors.extend(pc2.colors)
        pointcloud.points.extend(pc3.points)
        pointcloud.colors.extend(pc3.colors)
        pointcloud.points.extend(pc4.points)
        pointcloud.colors.extend(pc4.colors)

        # # remove the points in pointcloud that pertain to the top bit of z
        # pts = np.asarray(pointcloud.points)
        # colors = np.asarray(pointcloud.colors)
        # ind_z_top = np.where(pts[:, 2] < 0.12)
        # pointcloud.points = o3d.utility.Vector3dVector(pts[ind_z_top])
        # pointcloud.colors = o3d.utility.Vector3dVector(colors[ind_z_top])

        # TODO: have a dynamically changing script logic
            # 1) isolate the 4x point cloud like normal
            # 2) identify the max z and the min/max x and y of the point cloud
            # 3) remove the top z centimeter of the point cloud
            # 4) remove most of the points (severely downsample) in the center circle part of the point cloud (radius ~ 3cm?)
            # 5) isolate the ee point cloud seperately
            # 6) fuse the two point clouds together ---> test to ensure this works well with varying pottery situations 

        # add in the pc1
        pointcloud.points.extend(pc1.points)
        pointcloud.colors.extend(pc1.colors)

        # crop point cloud
        pointcloud, ind = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )  # remove statistical outliers
        # o3d.visualization.draw_geometries([pointcloud])
        pointcloud = self.remove_stage_grippers(pointcloud)
        # o3d.visualization.draw_geometries([pointcloud])
        pointcloud = self.remove_background(
            pointcloud, radius=0.15, center=np.array([0.6, 0.0, 0.15])# np.array([0.6, -0.05, 0.3])
        )
        # o3d.visualization.draw_geometries([pointcloud])

        # color thresholding
        pointcloud = self.lab_color_crop(pointcloud, color)
        pointcloud, ind = pointcloud.remove_radius_outlier(
            nb_points=20, radius=0.005
        ) 
        # o3d.visualization.draw_geometries([pointcloud])


        # get shape of clay base
        pointcloud.estimate_normals()
        downpdc = pointcloud.voxel_down_sample(voxel_size=0.0025)
        downpdc_points = np.asarray(downpdc.points)
        # o3d.visualization.draw_geometries([downpdc])

        # uniformly sample 2048 points from each point cloud
        points = np.asarray(downpdc.points)
        idxs = np.random.randint(0, len(points), size=2048)
        points = points[idxs]

        # re-process the processed_pcl to center
        # pc_center = np.array([0.6, 0.0, 0.24])
        pc_center = downpdc.get_center() 
        return points, pc_center

    
    def unnormalize_fuse_point_clouds_no_base(self, pc2, pc3, pc4, pc5, no_transformation=False, color='Green'):
        # start = time.time()
        if not no_transformation:
            # transform each cloud to world frame
            pc2.transform(self.camera_transforms[2])
            pc3.transform(self.camera_transforms[3])
            pc4.transform(self.camera_transforms[4])
            pc5.transform(self.camera_transforms[5])

        # combine the point clouds
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = pc5.points
        pointcloud.colors = pc5.colors
        pointcloud.points.extend(pc2.points)
        pointcloud.colors.extend(pc2.colors)
        pointcloud.points.extend(pc3.points)
        pointcloud.colors.extend(pc3.colors)
        pointcloud.points.extend(pc4.points)
        pointcloud.colors.extend(pc4.colors)

        # crop point cloud
        pointcloud, ind = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )  # remove statistical outliers
        # o3d.visualization.draw_geometries([pointcloud])
        pointcloud = self.remove_stage_grippers(pointcloud)
        # o3d.visualization.draw_geometries([pointcloud])
        pointcloud = self.remove_background(
            pointcloud, radius=0.15, center=np.array([0.6, 0.0, 0.15])# np.array([0.6, -0.05, 0.3])
        )
        # o3d.visualization.draw_geometries([pointcloud])

        # color thresholding
        pointcloud = self.lab_color_crop(pointcloud, color)
        pointcloud, ind = pointcloud.remove_radius_outlier(
            nb_points=20, radius=0.005
        ) 
        # o3d.visualization.draw_geometries([pointcloud])

        # print("test: ", np.asarray(pointcloud.points).shape)

        # print("1")
        # o3d.visualization.draw_geometries([pointcloud])

        # get shape of clay base
        pointcloud.estimate_normals()
        downpdc = pointcloud.voxel_down_sample(voxel_size=0.0025)
        downpdc_points = np.asarray(downpdc.points)
        # o3d.visualization.draw_geometries([downpdc])

        # uniformly sample 2048 points from each point cloud
        points = np.asarray(downpdc.points)
        idxs = np.random.randint(0, len(points), size=2048)
        points = points[idxs]

        # re-process the processed_pcl to center
        # pc_center = np.array([0.6, 0.0, 0.24])
        pc_center = downpdc.get_center() 
        return points, pc_center# start = time.time()
                                              
    
    def unnormalize_fuse_point_clouds(self, pc2, pc3, pc4, pc5, no_transformation=False, color='Green'):
        """
        Stitches together 4 point clouds from cameras 2, 3, 4, and 5 in the
        Franka workspace into world coordinate frame, performs background
        and color cropping, and scales up by 10 to adjust for size of point clouds
        in ShapeNet dataset

        Args:
        pc2 (o3d.geometry.PointCloud): point cloud from camera 2
        pc3 (o3d.geometry.PointCloud): point cloud from camera 3
        pc4 (o3d.geometry.PointCloud): point cloud from camera 4
        pc5 (o3d.geometry.PointCloud): point cloud from camera 5
        vis (bool): if True, will display final results (default False)

        Returns:
        unscaled, unnormalized cloud (o3d.geometry.PointCloud): stitched cloud
        """
        # start = time.time()
        if not no_transformation:
            # transform each cloud to world frame
            pc2.transform(self.camera_transforms[2])
            pc3.transform(self.camera_transforms[3])
            pc4.transform(self.camera_transforms[4])
            pc5.transform(self.camera_transforms[5])

        # combine the point clouds
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = pc5.points
        pointcloud.colors = pc5.colors
        pointcloud.points.extend(pc2.points)
        pointcloud.colors.extend(pc2.colors)
        pointcloud.points.extend(pc3.points)
        pointcloud.colors.extend(pc3.colors)
        pointcloud.points.extend(pc4.points)
        pointcloud.colors.extend(pc4.colors)

        # o3d.visualization.draw_geometries([pointcloud])

        # print("test: ", np.asarray(pointcloud.points).shape)

        # crop point cloud
        pointcloud, ind = pointcloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )  # remove statistical outliers
        # o3d.visualization.draw_geometries([pointcloud])
        pointcloud = self.remove_stage_grippers(pointcloud)
        # o3d.visualization.draw_geometries([pointcloud])
        pointcloud = self.remove_background(
            pointcloud, radius=0.15, center=np.array([0.6, 0.0, 0.15])# np.array([0.6, -0.05, 0.3])
        )

        # print("test: ", np.asarray(pointcloud.points).shape)

        # o3d.visualization.draw_geometries([pointcloud])

        # color thresholding
        pointcloud = self.lab_color_crop(pointcloud, color)
        pointcloud, ind = pointcloud.remove_radius_outlier(
            nb_points=20, radius=0.005
        ) 

        # print("test: ", np.asarray(pointcloud.points).shape)

        # print("1")
        # o3d.visualization.draw_geometries([pointcloud])

        # get shape of clay base
        pointcloud.estimate_normals()
        downpdc = pointcloud.voxel_down_sample(voxel_size=0.0025)
        downpdc_points = np.asarray(downpdc.points)
      
        # polygon_indices = np.where(downpdc_points[:,2] < 0.236) # PREVIOUS BEFORE 8/29
        # polygon_indices = np.where(downpdc_points[:, 2] < 0.22) # PREVIOUS BEFORE 11/30
        # polygon_indices = np.where(downpdc_points[:, 2] < 0.24) # PREVIOUS BEFORE 2/7/24

        polygon_indices = np.where(downpdc_points[:, 2] < 0.075) 
        polygon_pcl = o3d.geometry.PointCloud()
        polygon_pcl.points = o3d.utility.Vector3dVector(downpdc_points[polygon_indices])

        # print("2")
        # o3d.visualization.draw_geometries([polygon_pcl])

        # generate a 2d grid of points for the base
        base_plane = []
        minx, maxx = np.amin(downpdc_points[:, 0]), np.amax(downpdc_points[:, 0])
        miny, maxy = np.amin(downpdc_points[:, 1]), np.amax(downpdc_points[:, 1])
        minz, maxz = np.amin(downpdc_points[:, 2]), np.amax(downpdc_points[:, 2])
        x_vals = np.linspace(minx, maxx, 50)
        y_vals = np.linspace(miny, maxy, 50)
        xx, yy = np.meshgrid(
            x_vals, y_vals
        )  # create grid that covers full area of 2d polygon
        # z = 0.234 # height of the clay base

        z = 0.0675
        # print("\n\n\n\nMODIFIED!!!")
        # z = 0.21 # PREVIOUS BEFORE 11/30
        # z = 0.232 # PREVIOUS BEFORE 8/29
        zz = np.ones(len(xx.flatten())) * z
        points = np.vstack((xx.flatten(), yy.flatten(), zz)).T

        grid_cloud = o3d.geometry.PointCloud()
        grid_cloud.points = o3d.utility.Vector3dVector(points)

        # print("3")
        # o3d.visualization.draw_geometries([grid_cloud])

        # crop shape of clay base out of 2d grid of points
        polygon_coords = np.asarray(polygon_pcl.points)[:, 0:2]
        polygon = Polygon(polygon_coords)
        mask = [
            polygon.contains(Point(x, y))
            for x, y in np.asarray(grid_cloud.points)[:, 0:2]
        ]
        cropped_grid = np.asarray(grid_cloud.points)[:, 0:2][mask]
        zs = np.ones(len(cropped_grid)) * z
        cropped_grid = np.concatenate(
            (cropped_grid, np.expand_dims(zs, axis=1)), axis=1
        )

        base_cloud = o3d.geometry.PointCloud()
        base_cloud.points = o3d.utility.Vector3dVector(cropped_grid)

        # add top part of clay to new base
        base_cloud.points.extend(downpdc.points)
        cropped_plane, ind = base_cloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )

        base_cloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([0, 0, 1]), (len(base_cloud.points), 1))
        )

        # print("4")
        # o3d.visualization.draw_geometries([base_cloud])

        # uniformly sample 2048 points from each point cloud
        points = np.asarray(base_cloud.points)
        idxs = np.random.randint(0, len(points), size=2048)
        points = points[idxs]

        # re-process the processed_pcl to center
        # pc_center = np.array([0.6, 0.0, 0.24])
        pc_center = base_cloud.get_center() 
        return points, pc_center

    def fuse_point_clouds(self, pc2, pc3, pc4, pc5, vis=False, no_transformation=False):
        """
        Stitches together 4 point clouds from cameras 2, 3, 4, and 5 in the
        Franka workspace into world coordinate frame, performs background
        and color cropping, and scales up by 10 to adjust for size of point clouds
        in ShapeNet dataset

        Args:
        pc2 (o3d.geometry.PointCloud): point cloud from camera 2
        pc3 (o3d.geometry.PointCloud): point cloud from camera 3
        pc4 (o3d.geometry.PointCloud): point cloud from camera 4
        pc5 (o3d.geometry.PointCloud): point cloud from camera 5
        vis (bool): if True, will display final results (default False)

        Returns:
        rescaled_points (o3d.geometry.PointCloud): stitched and cropped final
        point cloud scaled up by 10
        """
        points, pc_center = self.unnormalize_fuse_point_clouds(pc2, pc3, pc4, pc5, no_transformation)
        centered_points = points - pc_center
        scale = 10
        rescaled_points = scale * centered_points

        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(rescaled_points)
        pointcloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([0, 0, 1]), (len(rescaled_points), 1))
        )

        if vis:
            o3d.visualization.draw_geometries([pointcloud])

        return rescaled_points
    
    def scale_point_clouds(self, points, pc_center):
        centered_points = points - pc_center
        scale = 10
        rescaled_points = scale * centered_points

        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(rescaled_points)
        pointcloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([0, 0, 1]), (len(rescaled_points), 1))
        )
        return rescaled_points
