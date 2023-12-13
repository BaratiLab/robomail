import numpy as np

from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, JointPositionSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

import rospy
import threading

from copy import deepcopy
import time

from typing import List, Dict, Tuple

def reset_joints():
    FrankaArm().reset_joints()

class GotoPoseLive:
    def __init__(self, dt = 0.04, T = 1000, cartesian_impedances = None, step_size = 0.05):
        """
        NOTE: This code has not been commented or riggerously tested yet. It works (I think), but be careful when using it!
        dt: the time inbetween each communication loop
        T: the total time the skill will be live for
        cartesian_impedances: list of cartesian impedaces. If none, uses default from FranaConstants
        step_size: max distance the controller will tell the robot to move in one step. If the distance to travel
            is greater, then the robot will be told to move step_size towards the goal (at that iteration)
        """
        self.dt = dt
        self.T = T
        self.cartesian_impedances = cartesian_impedances
        self.fa = FrankaArm()
        
        self.pose = self.fa.get_pose() # pose to send to the skill
        print(type(self.pose), "intialize0")
        self.goal_pose = deepcopy(self.pose) # pose the user provides
        print(type(self.goal_pose), "intialize1")
        self.running = False # flag used to stop the loop
        self.step_size = step_size

        self.initialize = True # flag used to initialize the skill
        self.init_time = 0 # time the skill was initialized
        self.rate = rospy.Rate(1/self.dt) #set the rospy rate.
        self.pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10) #used to update the skill
        self.message_id = 0 #id counter for the ross message

    def _step(self, goal_pose=None, current_pose=None, ros_sleep = True):
        """
        This method is used to step the robot control once. It is called by the run function every timestep. The user should NOT use this method.
        If the user wants to control the robot themselves, use the public alias step() method.

        :param goal_pose: the goal pose of the robot. If none, uses the goal pose set by the user via set_goal_pose().
        :param current_pose: the current pose of the robot. If none, uses gets the current pose via fa.
        :param ros_sleep: if true, sleeps at the end of the command to ensure the control frequency is 1/dt.
        """

        if current_pose is None:
            current_pose = self.fa.get_pose() 
        
        if goal_pose is not None:
            self.set_goal_pose(goal_pose)

        delta_motion = self.goal_pose.translation - current_pose.translation
        self.move_pose = deepcopy(self.goal_pose)
        if (np.linalg.norm(delta_motion) > self.step_size):
            self.move_pose.translation = (delta_motion/np.linalg.norm(delta_motion))*self.step_size + current_pose.translation
            
        if self.initialize: # start the skill
            if self.cartesian_impedances != None:
                self.fa.goto_pose(self.move_pose, duration=self.T, dynamic=True, buffer_time=10,
                    cartesian_impedances=self.cartesian_impedances)
            else:
                self.fa.goto_pose(self.move_pose, duration=self.T, dynamic=True, buffer_time=10)

            self.initialize = False # no longer initalizing
            self.init_time = rospy.Time.now().to_time() # get initial time.
        
        else: # send update message
            # print("update go to pose", self.pose.translation)
            timestamp = rospy.Time.now().to_time() - self.init_time # update time stamp
            
            # ross messages:
            traj_gen_proto_msg = PosePositionSensorMessage(
                id=self.message_id, timestamp=timestamp,
                position=self.move_pose.translation, quaternion=self.move_pose.quaternion
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
            )
            # uncomment below to log the ross messages to the terminal
            # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
            self.pub.publish(ros_msg) # send message to franka

        if ros_sleep:
            self.rate.sleep() #sleep for dt. This should only sleep if it is called within self.dt of the previous call.
        self.message_id += 1
    
    def step(self, goal_pose=None, current_pose=None, ros_sleep = True):
        """
        This method allows the user to run the controller one step at a time, instead of using asynchronous control.
        It is an alias for the private _step() method, with protections to ensure it is not called when the asynchronous control is running.
        
        :param goal_pose: the goal pose of the robot. If none, uses the goal pose set by the user via set_goal_pose().
        :param current_pose: the current pose of the robot. If none, uses gets the current pose via FrankaArm.
        :param ros_sleep: if true, sleeps at the end of the command to ensure the control frequency is 1/dt.
        """
        if self.running:
            raise Exception("Cannot call step() while the threaded controller is running. Stop the asynchronous control first by calling stop()")
        else:
            self._step(goal_pose, current_pose, ros_sleep)


    def _run(self):
        """
        Runs the skill. This is called when the start() method is called. Uses rospy rate.sleep() to maintain a control frequency of 1/dt.
        """
        while self.running:
            self._step(ros_sleep=True)

    def get_goal_pose(self):
        # Returns the goal pose
        return self.goal_pose
    
    def set_goal_pose(self, goal_pose):
        # set the goal pose. This is used to control the robot.
        self.goal_pose = goal_pose

    def set_goal_translation(self, goal_translation):
        # sets the goal pose translation, leaves the goal rotation unchanged.
        self.goal_pose.translation = goal_translation
    
    def start(self):
        """
        Runs the _step() function in a new thread, allowing for asynchronous control of the robot.
        The thread is terminated when stop() is called.
        """
        # start _run() in a new thread
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
    
    def stop(self):
        """
        Stops the thread running the _step() function.
        """
        # stop runing the thread by setting the 'running' flag to false, then waiting for 
        # the tread to terminate. Finally, stop any ongoing skill.
        self.running = False
        self.thread.join()
        print('joined')
        self.fa.stop_skill()
        self.initialize = False

    def get_pose(self):
        # Returns the current pose
        return self.fa.get_pose()


class GotoJointsLive:
    def __init__(self, dt:float = 0.04, T:int = 1000, k_gains:List[float] = (0.5*np.array(FC.DEFAULT_K_GAINS)).tolist(), 
                 d_gains:List[float] = (0.5*np.array(FC.DEFAULT_D_GAINS)).tolist(), ignore_virtual_walls:bool = False, 
                 max_rotation:float = 0.3) -> None:
        """
        dt: the time inbetween each communication loop (0.04 is about the min you want to do)
        T: the total time the skill will be live for
        k_gains: list of k gains for joint PID. Defaults to 50% of the defualts listed in Franka Constants (I found this value worked well)
        d_gains: list of d gains for joint PID. Defaults to 50% of the defualts listed in Franka Constants (I found this value worked well)
        max_rotation: maximum rotation the arm will command one joint to move in one timestep. 
        ignore_virtual_walls: ignores python virtual walls. We shouldn't need to do this but we do, for some reason.
        """
        self.dt: float = dt
        self.T: int = T
        self.k_gains: List[float] = k_gains
        self.d_gains: List[float] = d_gains

        self.fa = FrankaArm()
        self.goal_joints: np.ndarray = self.fa.get_joints() # joints the user provides
        self.running: bool = False # flag used to stop the loop
        self.ignore_virtual_walls: bool = ignore_virtual_walls
        self.max_rotation: float = max_rotation

        self.initialize: bool = True # flag used to initialize the skill
        self.init_time: float = 0 # time the skill was initialized
        self.message_id: int = 0 #id counter for the ross message
        self.rate: rospy.Rate = rospy.Rate(1/self.dt) #set the rospy rate.
        self.pub: rospy.Publisher = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10) #used to update the skill

    def _step(self, goal_joints:np.ndarray = None, current_joints:np.ndarray = None, ros_sleep:bool = True) -> None:
        """
        This method is used to step the robot control once. It is called by the run function every timestep. However, if the user
        wants to control the robot themselves, they can call this method to step the robot once.
        
        :param goal_joints: the goal joints of the robot. If none, uses the goal joints set by the user via set_goal_joints().
        :param current_joints: the current joints of the robot. If none, uses gets the current joints via FrankaArm.
        :param ros_sleep: if true, sleeps at the end of the command to
        """
        if current_joints is None:
            current_joints: np.ndarray = self.fa.get_joints()

        if goal_joints is not None:
            self.set_goal_joints(goal_joints)

        delta_joints: np.ndarray = self.goal_joints - current_joints
        move_joints: np.ndarray = deepcopy(self.goal_joints)
        for i in range(len(delta_joints)):
            if abs(delta_joints[i]) > self.max_rotation:
                move_joints[i] = self.max_rotation*np.sign(delta_joints[i]) + current_joints[i]
                print('violating max rotation')

        if self.initialize: # start the skill
            self.fa.goto_joints(joints=move_joints, duration=self.T, dynamic=True, ignore_virtual_walls=self.ignore_virtual_walls, 
                                k_gains=self.k_gains, d_gains=self.d_gains)

            self.initialize = False
            self.init_time: float = rospy.Time.now().to_time() # get initial time.
        
        else: # send update message
            # print("update go to pose", self.pose.translation)
            timestamp: float = rospy.Time.now().to_time() - self.init_time # update time stamp

            # ross messages:
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=self.message_id, timestamp=timestamp,
                joints = move_joints
            )
            ros_msg: SensorDataGroup = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    sensor_proto_msg=traj_gen_proto_msg, sensor_data_type=SensorDataMessageType.POSE_POSITION),
            )

            # uncomment below to log the ross messages to the terminal
            # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
            self.pub.publish(ros_msg)
        if ros_sleep:
            self.rate.sleep() # sleep for dt. This should only sleep if it is called within self.dt of the previous call.
        self.message_id += 1

    def step(self, goal_joints:np.ndarray = None, current_joints:np.ndarray = None, ros_sleep:bool = True) -> None:
        """
        This method allows the user to run the controller one step at a time, instead of using asynchronous control.
        It is an alias for the private _step() method, with protections to ensure it is not called when the asynchronous control is running.
        
        :param goal_joints: the goal joints of the robot. If none, uses the goal joints set by the user via set_goal_joints().
        :param current_joints: the current joints of the robot. If none, uses gets the current joints via FrankaArm.
        :param ros_sleep: if true, sleeps at the end of the command to ensure the control frequency is 1/dt.
        """
        if self.running:
            raise Exception("Cannot call step() while the threaded controller is running. Stop the asynchronous control first by calling stop()")
        else:
            self._step(goal_joints, current_joints, ros_sleep)


    def _run(self) -> None:  
        """
        Runs the skill. This is called when the start() method is called. Uses rospy rate.sleep() to maintain a control frequency of 1/dt.
        """
        while self.running:
            self._step(ros_sleep=True)

    def get_goal_joints(self) -> np.ndarray:
        # Returns the goal joints
        return self.goal_joints
    
    def set_goal_joints(self, goal_joints) -> None:
        # set the goal joints. This is used to control the robot.
        self.goal_joints = goal_joints
        
            
    def start(self) -> None:
        # start _run() in a new thread
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    
    def stop(self) -> None:
        # stop runing the thread by setting the 'running' flag to false, then waiting for 
        # the tread to terminate. Finally, stop any ongoing skill.
        self.running = False
        self.thread.join()
        print('joined')
        self.fa.stop_skill()

if __name__ == "__main__":
    # go to pose test
    controller = GotoPoseLive()
    start_time = time.time()
    goal_pose = deepcopy(FC.HOME_POSE)

    controller.start()
    t = 0
    while t < 20: 
        t = time.time() - start_time   
        x = 0.5 + 0.1*np.sin(t)
        y = 0.1*np.cos(t/1.5)
        z = 0.4
        goal_pose.translation = np.array([x,y,z])
        time.sleep(np.random.uniform(0.01, 0.1))
        controller.set_goal_pose(goal_pose)
        # time.sleep(1)
        # current_pose = controller.fa.get_pose()
        # controller.step(goal_pose=goal_pose, current_pose=current_pose)
    controller.stop()


