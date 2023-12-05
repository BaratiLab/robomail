import copy
import numpy as np
import open3d as o3d


class CalibrationClass:
    def __init__(self):
        pass

    def draw_registration_result(self, source, target, transformation):
        """
        Visualizes a transformed COPY of the source point cloud together with the original target point cloud
        NOTE: This function does not apply a transform to the original source or target pointclouds

        Args:
        source (o3d.geometry.PointCloud): source point cloud
        target (o3d.geometry.PointCloud): target point cloud
        transformation (npt.NDArray): transformation matrix of size (4,4)
        """

        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])

    def preprocess_point_cloud(self, pcd, voxel_size):
        """
        Downsamples input point cloud based on a given voxel size and computes FPFH feature

        Args:
        pcd (o3d.geometry.PointCloud): input point cloud
        voxel_size (float): voxel size to downsample into

        Returns:
        pcd_down (o3d.geometry.PointCloud): output downsampled point cloud
        pcd_fpfh (open3d.registration.Feature): FPFH feature of input point cloud
        """
        print("\nDownsampling with a voxel size %.3f..." % voxel_size)
        pcd_down = pcd.voxel_down_sample(
            voxel_size * 0.5
        )  # global registration is only performed on a heavily down-sampled point clouds

        radius_normal = voxel_size * 2
        print("\nEstimating normal with search radius %.3f..." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
        )

        radius_feature = voxel_size * 5
        print("\nComputing FPFH feature with search radius %.3f...  " % radius_feature)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100),
        )

        return pcd_down, pcd_fpfh

    def prepare_dataset(self, voxel_size, source, target, transform=np.identity(4)):
        """
        Preprocesses source and target point clouds using preprocess_point_cloud()

        Args:
        voxel_size (float): voxel size to downsample into
        source (o3d.geometry.PointCloud): source point cloud
        target (o3d.geometry.PointCloud): target point cloud
        transform (npt.NDArray): transformation matrix of size (4,4), default identity matrix (no transformation)

        Returns:
        source (o3d.geometry.PointCloud): source point cloud
        target (o3d.geometry.PointCloud): target point cloud
        source_down (o3d.geometry.PointCloud): downsampled source point cloud
        target_down (o3d.geometry.PointCloud): downsampled target point cloud
        source_fpfh (open3d.registration.Feature): FPFH feature of source point cloud
        target_fpfh (open3d.registration.Feature): FPFH feature of target point cloud
        """

        print("\nDisplaying raw alignment...")
        self.draw_registration_result(source, target, transform)

        source_down, source_fpfh = self.preprocess_point_cloud(
            source, voxel_size * 3
        )  # compute FPFH and downsampled point cloud for source and target
        target_down, target_fpfh = self.preprocess_point_cloud(target, voxel_size * 3)

        return source, target, source_down, target_down, source_fpfh, target_fpfh

    def execute_global_registration(
        self, source_down, target_down, source_fpfh, target_fpfh, voxel_size
    ):
        """
        Applies RANSAC global registration method to align point clouds from different cameras

        Args:
        source_down (o3d.geometry.PointCloud): downsampled source point cloud from prepare_dataset()
        target_down (o3d.geometry.PointCloud): downsampled target point cloud from prepare_dataset()
        source_fpfh (o3d.registration.Feature): FPFH feature for each point of source point cloud
        target_fpfh (o3d.registration.Feature): FPFH feature for each point of target point cloud
        voxel_size (float): voxel size used to define distance threshold for alignment

        Returns:
        result (o3d.pipelines.registration.RegistrationResult): output of Open3D global RANSAC registration function
        """
        distance_threshold = voxel_size * 1.5  # define maximum correspondence distance
        print("\nGlobal registration with RANSAC on downsampled point clouds...")
        print("\nDownsampling voxel size: %.3f," % voxel_size)
        print("\nDistance threshold: %.3f." % distance_threshold)

        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source=source_down,
            target=target_down,
            source_feature=source_fpfh,
            target_feature=target_fpfh,
            mutual_filter=True,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
                True
            ),
            ransac_n=4,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold
                ),
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                max_iteration=4000000, confidence=1.0
            ),
        )

        # visualize
        print("\nDisplaying alignment after RANSAC global registration...")
        self.draw_registration_result(source_down, target_down, result.transformation)

        return result

    def refine_registration(self, source, target, distance_threshold, trans_global):
        """
        Applies iterative-closest point algorithm (ICP) to refine global registration
        NOTE: Global registration should be run on source and target point clouds before using
        this function

        Args:
        source (o3d.geometry.PointCloud): source point cloud
        target (o3d.geometry.PointCloud): target point cloud
        distance_threshold (float): distance threshold for alignment
        trans_global (npt.NDArray): result of global registration, matrix of size (4,4), can use result.transformation

        Returns:
        result (o3d.pipelines.registration.RegistrationResult): output of Open3D ICP registration function
        """
        # calculate global registration evaluation
        print("\nGlobal registration alignment evaluation:")
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, distance_threshold, trans_global
        )
        print(evaluation)

        print("\nPoint-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % distance_threshold)

        result = o3d.pipelines.registration.registration_icp(
            source=source,
            target=target,
            max_correspondence_distance=distance_threshold,
            init=trans_global,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        )
        # criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10))

        # visualize
        print("\nDisplaying alignment after local refinement...")
        self.draw_registration_result(source, target, result.transformation)

        return result
