from robomail.vision.camera import CameraClass
from multiprocessing import Process, Queue, Event, Manager
import numpy as np
import time
from copy import deepcopy

def test_func(frame_que, stop_running_event):
    count = 0
    while not stop_running_event.is_set():
        frame_que.put(count)
        count += 1
        time.sleep(0.01)
    print('test process stopped')
    frame_que.put(None) # put a None to signal the end of the queue

def run_cameras(cam_numbers, image_heights, image_widths, pointcloud_flags, verts_flags, frame_que, stop_running_event):
    cameras = []
    for i, cam_num in enumerate(cam_numbers):
        cameras.append(CameraClass(cam_num, H=image_heights[i], W=image_widths[i]))
    counter = 0
    while not stop_running_event.is_set():
        frame_data = []
        # Empty array to store new data
        for i, camera in enumerate(cameras):
            frame_data.append(camera.get_next_frame(get_point_cloud=pointcloud_flags[i], get_verts=verts_flags[i]))
        # frame_que.put(frame_data)
        print('start put')
        frame_que.put(frame_data)
        counter += 1
        print('end put')
    print('camera process stopped')
    frame_que.put(None) # put a None to signal the end of the queue


class ThreadedCameras:
    def __init__(self, cam_numbers, image_height=480, image_width=848, get_point_cloud=True, get_verts=True):
        """
        Initializes ThreadedCameras.

        :param cam_numbers: List of camera numbers to initialize CameraClass instances.
        :param image_height: Height of the captured image. Defaults to 480. Either int (all cameras have same height) or list (each camera has its own height).
        :param image_width: Width of the captured image. Defaults to 848. Either int (all cameras have same width) or list (each camera has its own width).
        :param get_point_cloud: Flag to retrieve point cloud data. Defaults to False. Either bool (all cameras have same flag) or list (each camera has its own flag).
        :param get_verts: Flag to retrieve vertices data. Defaults to False. Either bool (all cameras have same flag) or list (each camera has its own flag).
        """

        # Check the format of the input parameters, and if the parameters are single values, make them into a
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

        self._frame_que = Manager().Queue()
        self._stop_running = Event()

        self._process = Process(target=run_cameras, args=(cam_numbers, image_heights, image_widths, self.get_point_cloud, 
                                                          self.get_verts, self._frame_que, self._stop_running), daemon=True)
        self._process.start()
        self._last_frame = None
        self._frame_index = 0

    def get_frame(self):
        """
        Retrieves the latest captured frame from a camera.

        :param cam_number: Camera number to retrieve frame from.
        :return: A copy of the captured frame.
        """
        while not self._frame_que.empty():
            self._last_frame = self._frame_que.get_nowait()
            self._frame_index += 1
            
        return deepcopy(self._last_frame)
            
        
    def get_next_frame(self):
        """
        Waits for a new set of frames to be captured and retrieves them.
        :return: A copy of the captured frames.
        """
        # clear the queue of old frames:
        while not self._frame_que.empty():
            self._frame_que.get_nowait()
            self._frame_index += 1
        
        # wait for a new frame
        self._last_frame = self._frame_que.get()
        self._frame_index += 1
        return deepcopy(self._last_frame)         
    

    def get_frame_index(self):
        """
        Returns the current frame index.
        :return: The current frame index.
        """
        start_time = time.time()
        # If there are frames in the queue, update _last_frame and _frame_index
        while not self._frame_que.empty():
            update_frame_start = time.time()
            self._last_frame = self._frame_que.get_nowait()
            print('update_frame time: ', time.time() - update_frame_start)
            self._frame_index += 1
        print('get_frame_index time: ', time.time() - start_time)
        return self._frame_index

    def stop(self):
        """Stops the thread and joins it, ending the frame capturing process."""
        self._stop_running.set()
        # Flush the queue
        while self._frame_que.get() is not None:
            pass
        self._process.join()
        print('threaded cameras stopped')
    
if __name__ == "__main__":
    import cv2
    cam_numbers = [1, 2, 3, 4, 5]
    threaded_cameras = ThreadedCameras(cam_numbers, get_point_cloud=False, get_verts=False)
    last_frame_index = 0
    for i in range(200):
        time.sleep(1)
        last_time = time.time()
        print("Frame " + str(i))
        if threaded_cameras.get_frame_index() > last_frame_index:
            frames = threaded_cameras.get_frame()
        else:
            frames = threaded_cameras.get_next_frame()
            print('running next frame')
        print("Frame index: ", threaded_cameras.get_frame_index())
        last_frame_index = threaded_cameras.get_frame_index()
        # print(frames)
        for i, frame in enumerate(frames):
            img, depth, pointcloud, verts = frame
            cv2.imshow("Image " + str(i), img)
        cv2.waitKey(1)
        print("Time: " + str(time.time() - last_time))
    threaded_cameras.stop()
    # quit_event = Event()
    # que = Queue()
    # p = Process(target=test_func, args=(que, quit_event), daemon=True)

    # p.start()
    # for i in range(100):
    #     while not que.empty():
    #         print(que.get_nowait())
    #     time.sleep(1)
    