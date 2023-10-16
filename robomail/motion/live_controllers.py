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


    def run(self):
        print('start run')
        rate = rospy.Rate(1/self.dt) #set the rospy rate.
        pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10) #used to update the skill
        initialize = True #set initilize to true
        id = 0 #id counter for the ross message

        # loop untill the 'running' flag is set to false
        while self.running:
            # If the robot is trying to move more than step_size, clip the motion.
            cur_pose = self.fa.get_pose() 
            delta_motion = self.goal_pose.translation - cur_pose.translation
            self.pose = deepcopy(self.goal_pose)
            if (np.linalg.norm(delta_motion) > self.step_size):
                self.pose.translation = (delta_motion/np.linalg.norm(delta_motion))*self.step_size + cur_pose.translation
                

            if initialize: # start the skill
                if self.cartesian_impedances != None:
                    self.fa.goto_pose(self.pose, duration=self.T, dynamic=True, buffer_time=10,
                        cartesian_impedances=self.cartesian_impedances)
                else:
                    self.fa.goto_pose(self.pose, duration=self.T, dynamic=True, buffer_time=10)

                initialize = False # no longer initalizing
                init_time = rospy.Time.now().to_time() # get initial time.
            
            else: # send update message
                # print("update go to pose", self.pose.translation)
                timestamp = rospy.Time.now().to_time() - init_time # update time stamp
                
                # ross messages:
                traj_gen_proto_msg = PosePositionSensorMessage(
                    id=id, timestamp=timestamp,
                    position=self.pose.translation, quaternion=self.pose.quaternion
                )
                ros_msg = make_sensor_group_msg(
                    trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                        traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
                )
                # uncomment below to log the ross messages to the terminal
                # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
                pub.publish(ros_msg) # send message to franka
            rate.sleep() #sleep for dt.
            id += 1

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
        # start run() in a new thread
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
    
    def stop(self):
        # stop runing the thread by setting the 'running' flag to false, then waiting for 
        # the tread to terminate. Finally, stop any ongoing skill.
        self.running = False
        self.thread.join()
        print('joined')
        self.fa.stop_skill()

    def get_pose(self):
        # Returns the current pose
        return self.fa.get_pose()
    

# class GotoPoseGripperLive:
#     def __init__(self, dt = 0.04, T = 1000, cartesian_impedances = None, step_size = 0.05, min_gripper_distance = 0.03875):
#         """
#         NOTE: This code has not been commented or riggerously tested yet. It works (I think), but be careful when using it!
#         dt: the time inbetween each communication loop
#         T: the total time the skill will be live for
#         cartesian_impedances: list of cartesian impedaces. If none, uses default from FranaConstants
#         step_size: max distance the controller will tell the robot to move in one step. If the distance to travel
#             is greater, then the robot will be told to move step_size towards the goal (at that iteration)
#         """
#         self.dt = dt
#         self.T = T
#         self.cartesian_impedances = cartesian_impedances
#         self.fa = FrankaArm()
#         self.FC = FC
#         self.pose = self.fa.get_pose() # pose to send to the skill
#         self.goal_pose = deepcopy(self.pose) # pose the user provides
#         self.running = False # flag used to stop the loop
#         self.step_size = step_size 
#         self.gripper_width = deepcopy(self.fa.get_gripper_width())
#         self.new_gripper_command = False
#         self.grasping = False
#         self.min_gripper_distance = min_gripper_distance # size of the object, to know when to use 'grasp = true'
#         self.grasp_command = {'grasp': False, 'width': min_gripper_distance}
#         self.max_gripper_width = 0.08


#     def run(self):
#         print('start run')
#         rate = rospy.Rate(1/self.dt) #set the rospy rate.
#         pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10) #used to update the skill
#         initialize = True #set initilize to true
#         id = 0 #id counter for the ross message

#         # loop untill the 'running' flag is set to false
#         while self.running:
#             # If the robot is trying to move more than step_size, clip the motion.
#             cur_pose = self.fa.get_pose() 
#             delta_motion = self.goal_pose.translation - cur_pose.translation
#             self.pose = deepcopy(self.goal_pose)
#             if (np.linalg.norm(delta_motion) > self.step_size):
#                 self.pose.translation = (delta_motion/np.linalg.norm(delta_motion))*self.step_size + cur_pose.translation
                
#             if self.new_gripper_command:
#                 print("gripper command", self.grasp_command['width'], self.grasp_command['grasp'])
#                 self.fa.goto_gripper(width=self.grasp_command['width'], grasp=self.grasp_command['grasp'], block=False, speed = 0.2)
#                 self.new_gripper_command = False

#             if initialize: # start the skill
#                 if self.cartesian_impedances != None:
#                     self.fa.goto_pose(self.pose, duration=self.T, dynamic=True, buffer_time=10,
#                         cartesian_impedances=self.cartesian_impedances)
#                 else:
#                     self.fa.goto_pose(self.pose, duration=self.T, dynamic=True, buffer_time=10)

#                 initialize = False # no longer initalizing
#                 init_time = rospy.Time.now().to_time() # get initial time.
            
#             else: # send update message
#                 # print("update go to pose", self.pose.translation)
#                 timestamp = rospy.Time.now().to_time() - init_time # update time stamp
                
#                 # ross messages:
#                 traj_gen_proto_msg = PosePositionSensorMessage(
#                     id=id, timestamp=timestamp,
#                     position=self.pose.translation, quaternion=self.pose.quaternion
#                 )
#                 ros_msg = make_sensor_group_msg(
#                     trajectory_generator_sensor_msg=sensor_proto2ros_msg(
#                         traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
#                 )
#                 # uncomment below to log the ross messages to the terminal
#                 # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
#                 pub.publish(ros_msg) # send message to franka
#             rate.sleep() #sleep for dt.
#             id += 1

#     def get_goal_pose(self):
#         # Returns the goal pose
#         return self.goal_pose
    
#     def set_goal_pose(self, goal_pose):
#         # set the goal pose. This is used to control the robot.
#         self.goal_pose = goal_pose

#     def set_goal_translation(self, goal_translation):
#         # sets the goal pose translation, leaves the goal rotation unchanged.
#         self.goal_pose.translation = goal_translation
    
#     def start(self):
#         # start run() in a new thread
#         self.running = True
#         self.thread = threading.Thread(target=self.run, daemon=True)
#         self.thread.start()
    
#     def stop(self):
#         # stop runing the thread by setting the 'running' flag to false, then waiting for 
#         # the tread to terminate. Finally, stop any ongoing skill.
#         self.running = False
#         self.thread.join()
#         print('joined')
#         self.fa.stop_skill()

#     def get_pose(self):
#         # Returns the current pose
#         return self.fa.get_pose()

#     def set_gripper_width(self, gripper_width:float):
#         # sets the gripper pose. This is used to control the robot.
#         if gripper_width == self.gripper_width:
#             return
        
#         self.new_gripper_command = True

#         if gripper_width < self.min_gripper_distance:
#             if not self.grasping:
#                 self.grasp_command = {'grasp': False, 'width': self.min_gripper_distance}
#                 self.gripper_width = self.min_gripper_distance
#                 self.grasping = True
#             else:
#                 self.new_gripper_command = False
        
#         elif gripper_width > self.max_gripper_width:
#             self.grasping = False
#             self.grasp_command = {'grasp': False, 'width': self.max_gripper_width}
#             self.gripper_width = self.max_gripper_width
#         else:
#             self.grasping = False
#             self.grasp_command = {'grasp': False, 'width': gripper_width}
#             self.gripper_width = gripper_width

#     def get_gripper_width(self):
#         # Returns the current gripper width
#         return self.gripper_width

class GotoJointsLive:
    def __init__(self, dt:float = 0.04, T:int = 1000, k_gains:List[float] = (0.5*np.array(FC.DEFAULT_K_GAINS)).tolist(), 
                 d_gains:List[float] = (0.5*np.array(FC.DEFAULT_D_GAINS)).tolist(), ignore_virtual_walls:bool = False, 
                 max_rotation:float = 0.3, max_speed:float = 0.5) -> None:
        """
        dt: the time inbetween each communication loop (0.04 is about the min you want to do)
        T: the total time the skill will be live for
        k_gains: list of k gains for joint PID. Defaults to 50% of the defualts listed in Franka Constants (I found this value worked well)
        d_gains: list of d gains for joint PID. Defaults to 50% of the defualts listed in Franka Constants (I found this value worked well)
        max_rotation: maximum rotation the arm will command one joint to move in one timestep. 
        max_speed: (rad/s) If the commanded rotation is greated then max_rotation, then the skill will have the arm rotate at this speed.
        ignore_virtual_walls: ignores python virtual walls. We shouldn't need to do this but we do, for some reason.
        """
        self.dt: float = dt
        self.T: int = T
        self.k_gains: List[float] = k_gains
        self.d_gains: List[float] = d_gains

        self.fa = FrankaArm()
        self.move_joints: np.ndarray = self.fa.get_joints() # joints to send to the skill
        self.goal_joints: np.ndarray = deepcopy(self.move_joints) # joints the user provides
        self.running: bool = False # flag used to stop the loop
        self.ignore_virtual_walls: bool = ignore_virtual_walls
        self.max_rotation: float = max_rotation
        self.max_speed: float = max_speed

    
    def run(self) -> None:  
        rate = rospy.Rate(hz=1/self.dt) #set the rospy rate.
        pub = rospy.Publisher(name=FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, data_class=SensorDataGroup, queue_size=10)
        initialize: bool = True #set initilize to true
        id: int = 0 #id counter for the ross message

        # loop untill the 'running' flag is set to false
        while self.running:
            # If the robot is trying to move more than max_rotation, clip the motion.
            cur_joints: np.ndarray = self.fa.get_joints()
            move_to_joints:np.ndarray = self.get_move_to_joints()
            delta_joints: np.ndarray = move_to_joints - cur_joints
            for i in range(len(delta_joints)):
                if abs(delta_joints[i]) > self.max_rotation:
                    move_to_joints[i] = self.max_rotation*np.sign(delta_joints[i]) + cur_joints[i]
                    print('violating max rotation')

            if initialize: # start the skill
                self.fa.goto_joints(joints=move_to_joints, duration=self.T, dynamic=True, ignore_virtual_walls=self.ignore_virtual_walls, 
                                    k_gains=self.k_gains, d_gains=self.d_gains)

                initialize = False
                init_time: float = rospy.Time.now().to_time() # get initial time.
            
            else: # send update message
                # print("update go to pose", self.pose.translation)
                timestamp: float = rospy.Time.now().to_time() - init_time # update time stamp

                # ross messages:
                traj_gen_proto_msg = JointPositionSensorMessage(
                    id=id, timestamp=timestamp,
                    joints = move_to_joints
                )
                ros_msg: SensorDataGroup = make_sensor_group_msg(
                    trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                        sensor_proto_msg=traj_gen_proto_msg, sensor_data_type=SensorDataMessageType.POSE_POSITION),
                )

                # uncomment below to log the ross messages to the terminal
                # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
                pub.publish(ros_msg)
            rate.sleep() # sleep for dt.
            id += 1

    def get_goal_joints(self) -> np.ndarray:
        # Returns the goal joints
        return self.goal_joints
    
    def set_goal_joints(self, goal_joints) -> None:
        # set the goal joints. This is used to control the robot.
        self.goal_joints = goal_joints
        self.start_joints: np.ndarray= self.fa.get_joints()
        self.move_start_time: float = time.time()

        # If the goal joint is futher away then a single time step movign at the max speed,
        # we will need to interpolate the move_to commands
        self.interpolate_move_to: bool = np.any(abs(self.goal_joints-self.start_joints) > self.max_rotation)


    def get_move_to_joints(self) -> np.ndarray:
        if self.interpolate_move_to:         
            delta_joints: np.ndarray = self.goal_joints - self.start_joints
            max_move: float = (time.time()-self.move_start_time)*self.max_speed
            motion_magnitude: np.ndarray = np.minimum(np.abs(delta_joints), max_move) # eliment-wize minimum
            return(self.start_joints + motion_magnitude*np.sign(delta_joints))
        else:
            return(self.goal_joints)
        
            
    def start(self) -> None:
        # start run() in a new thread
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        self.move_start_time = time.time()

    
    def stop(self) -> None:
        # stop runing the thread by setting the 'running' flag to false, then waiting for 
        # the tread to terminate. Finally, stop any ongoing skill.
        self.running = False
        self.thread.join()
        print('joined')
        self.fa.stop_skill()
        