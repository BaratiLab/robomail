from .camera import CameraClass
import threading
import numpy as np

class ThreadedCameras:
    """
    Manages multiple camera instances in a separate thread to capture frames asynchronously.
    Although asyncronus, the frames are synchronized.
    """

    def __init__(self, cam_numbers, image_height=480, image_width=848, get_point_cloud=True, get_verts=True):
        """
        Initializes ThreadedCameras.

        :param cam_numbers: List of camera numbers to initialize CameraClass instances.
        :param image_height: Height of the captured image. Defaults to 480. Either int (all cameras have same height) or list (each camera has its own height).
        :param image_width: Width of the captured image. Defaults to 848. Either int (all cameras have same width) or list (each camera has its own width).
        :param get_point_cloud: Flag to retrieve point cloud data. Defaults to False. Either bool (all cameras have same flag) or list (each camera has its own flag).
        :param get_verts: Flag to retrieve vertices data. Defaults to False. Either bool (all cameras have same flag) or list (each camera has its own flag).
        """
        self.cameras = []
        if isinstance(image_height, list) or isinstance(image_height, np.ndarray):
            assert len(image_height) == len(cam_numbers), "image_height must be either an int or a list of length len(cam_numbers)"
        else:
            image_heights = [image_height for _ in range(len(cam_numbers))]
        
        if isinstance(image_width, list) or isinstance(image_width, np.ndarray):
            assert len(image_width) == len(cam_numbers), "image_width must be either an int or a list of length len(cam_numbers)"
        else:
            image_widths = [image_width for _ in range(len(cam_numbers))]
        
        if isinstance(get_point_cloud, list) or isinstance(get_point_cloud, np.ndarray):
            assert len(get_point_cloud) == len(cam_numbers), "get_point_cloud must be either a bool or a list of length len(cam_numbers)"
        else:
            self.get_point_cloud = [get_point_cloud for _ in range(len(cam_numbers))]
        
        if isinstance(get_verts, list) or isinstance(get_verts, np.ndarray):
            assert len(get_verts) == len(cam_numbers), "get_verts must be either a bool or a list of length len(cam_numbers)"
        else:
            self.get_verts = [get_verts for _ in range(len(cam_numbers))]

        
        for i, cam_number in enumerate(cam_numbers):
            self.cameras.append(CameraClass(cam_number, H=image_heights[i], W=image_widths[i]))
        self._image_data = [None for _ in range(len(cam_numbers))]
        self._image_lock = threading.Lock()  # Lock for when self._image_data is being altered
        self.new_image_event = threading.Event()  # Event to signal when a new image is available
        self.record = False  # Flag to determine whether to record images
        self._thread = threading.Thread(target=self._run, daemon=True, )
        print('start thread')
        self._thread.start()

        
    def _run(self):
        """Internal method to continuously capture frames from cameras."""
        self.record = True 
        while self.record:
            # Empty array to store new data
            new_data = [None for _ in range(len(self.cameras))]

            for i, camera in enumerate(self.cameras):
                new_data[i] = camera.get_next_frame(get_point_cloud=self.get_point_cloud[i], get_verts=self.get_verts[i])

            with self._image_lock:
                # self._image_data now points to the recently collected data
                self._image_data = new_data
                self.new_image_event.set()

    def get_frames(self):
        """
        Retrieves the latest captured frames from the cameras.

        Returns:
        :return: A copy of the captured frames.
        """
        with self._image_lock:
            return self._image_data.copy()
        
    def get_next_frames(self):
        """
        Waits for a new set of frames to be captured and retrieves them.

        Returns:
        :return: A copy of the captured frames.
        """
        self.new_image_event.wait()
        with self._image_lock:
            self.new_image_event.clear()
            return self._image_data.copy()
        
    def stop(self):
        """Stops the thread and joins it, ending the frame capturing process."""
        self.record = False
        self._thread.join()
        print("camera thread joined")
        return
    

class NonThreadedCameras:
    """
    Manages multiple camera instances WITHOUT threading. Has the same interface as ThreadedCameras.
    Used to replace ThreadedCameras when debugging.
    """
    def __init__(self, cam_numbers, image_height=480, image_width=848, get_point_cloud=True, get_verts=True):
        """
        Initializes ThreadedCameras.

        :param cam_numbers: List of camera numbers to initialize CameraClass instances.
        :param image_height: Height of the captured image. Defaults to 480. Either int (all cameras have same height) or list (each camera has its own height).
        :param image_width: Width of the captured image. Defaults to 848. Either int (all cameras have same width) or list (each camera has its own width).
        :param get_point_cloud: Flag to retrieve point cloud data. Defaults to False. Either bool (all cameras have same flag) or list (each camera has its own flag).
        :param get_verts: Flag to retrieve vertices data. Defaults to False. Either bool (all cameras have same flag) or list (each camera has its own flag).
        """
        self.cameras = []
        if isinstance(image_height, list) or isinstance(image_height, np.ndarray):
            assert len(image_height) == len(cam_numbers), "image_height must be either an int or a list of length len(cam_numbers)"
        else:
            image_heights = [image_height for _ in range(len(cam_numbers))]
        
        if isinstance(image_width, list) or isinstance(image_width, np.ndarray):
            assert len(image_width) == len(cam_numbers), "image_width must be either an int or a list of length len(cam_numbers)"
        else:
            image_widths = [image_width for _ in range(len(cam_numbers))]
        
        if isinstance(get_point_cloud, list) or isinstance(get_point_cloud, np.ndarray):
            assert len(get_point_cloud) == len(cam_numbers), "get_point_cloud must be either a bool or a list of length len(cam_numbers)"
        else:
            self.get_point_cloud = [get_point_cloud for _ in range(len(cam_numbers))]
        
        if isinstance(get_verts, list) or isinstance(get_verts, np.ndarray):
            assert len(get_verts) == len(cam_numbers), "get_verts must be either a bool or a list of length len(cam_numbers)"
        else:
            self.get_verts = [get_verts for _ in range(len(cam_numbers))]

        
        for i, cam_number in enumerate(cam_numbers):
            self.cameras.append(CameraClass(cam_number, H=image_heights[i], W=image_widths[i]))


    def get_frames(self):
        """
        Gets images from all cameras and returns them. Same as get_next_frames().
        :return: A copy of the captured frames.
        """
        # Empty array to store new data
        new_data = [None for _ in range(len(self.cameras))]

        for i, camera in enumerate(self.cameras):
            new_data[i] = camera.get_next_frame(get_point_cloud=self.get_point_cloud[i], get_verts=self.get_verts[i])
        
        return new_data
        
    def get_next_frames(self):
        """
        Gets images from all cameras and returns them. Same as get_frames().
        :return: A copy of the captured frames.
        """
        return self.get_frames()
        
    def stop(self):
        """Does nothing."""
        pass