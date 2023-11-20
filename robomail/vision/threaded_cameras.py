from .camera import CameraClass
import threading

class ThreadedCameras:
    """
    Manages multiple camera instances in separate threads to capture frames asynchronously.
    Although asyncronus, the frames are synchronized.
    """

    def __init__(self, cam_numbers, image_height=480, image_width=848, get_point_cloud=True, get_verts=True):
        """
        Initializes ThreadedCameras.

        :param cam_numbers: List of camera numbers to initialize CameraClass instances.
        :param image_height: Height of the captured image. Defaults to 480.
        :param image_width: Width of the captured image. Defaults to 848.
        :param get_point_cloud: Flag to retrieve point cloud data. Defaults to False.
        :param get_verts: Flag to retrieve vertices data. Defaults to False.
        """
        self.cameras = []
        for cam_number in cam_numbers:
            self.cameras.append(CameraClass(cam_number, H=image_height, W=image_width))
        self._image_data = [None for _ in range(len(cam_numbers))]
        self._image_lock = threading.Lock()  # Lock for when self._image_data is being altered
        self.new_image_event = threading.Event()  # Event to signal when a new image is available
        self.record = False  # Flag to determine whether to record images
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_point_cloud = get_point_cloud
        self.get_verts = get_verts

        
    def _run(self):
        """Internal method to continuously capture frames from cameras."""
        self.record = True 
        while self.record:
            # Empty array to store new data
            new_data = [None for _ in range(len(self.cameras))]

            for i, camera in enumerate(self.cameras):
                new_data[i] = camera.get_next_frame(get_point_cloud=self.get_point_cloud, get_verts=self.get_verts)

            with self._image_lock:
                # self._image_data now points to the recently collected data
                self._image_data = new_data
                self.new_image_event.set()

    def get_frames(self):
        """
        Retrieves the latest captured frames from cameras.

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
        print("thread joined")
        return