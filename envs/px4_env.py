#!/usr/bin/env python
from __future__ import division

import gym
import time
import numpy as np
import socket
import subprocess
import os
import pickle
import types
from cmath import nan
from threading import Thread

from gym import utils, spaces

from . import gazebo_env
#from geometry_msgs.msg import Twist
#from std_srvs.srv import Empty

# ROS 2
import rclpy


from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # Used for publishing mara joint angles.
#from control_msgs.msg import JointTrajectoryControllerState
# from gazebo_msgs.srv import SetEntityState, DeleteEntity
#from gazebo_msgs.msg import ContactState, ModelState#, GetModelList
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from ros2pkg.api import get_prefix_path
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from gym.utils import seeding

from ROS.publishers import OffboardControlPublisher
from ROS.publishers import PositionPublisher
from ROS.publishers import VehicleCommandPublisher
from ROS.publishers import GymNode
from ROS.subscribers import TimesyncSubscriber
#different commands guide:
#176 - set mode 
# 400 - Arms / Disarms a component |1 to arm, 0 to disarm
#check https://github.com/PX4/PX4-Autopilot/blob/6ed48ad0c0bea36a1f94245962b4b6721553b310/msg/vehicle_command.msg for more

class SinglePx4UavEnv(gazebo_env.GazeboEnv):

    def __init__(self,args = None):
        self.des = [170, 0, 15]

        gazebo_env.GazeboEnv.__init__(self, "/home/user/landing/gym_gazebo2_px4/launch_files/px4_sim_launch.py")
        #rospy.wait_for_service('/gazebo/unpause_physics', 30)
        #self.unpause = rclpy.ServiceProxy('/gazebo/unpause_physics', Empty)
        #self.pause = rclpy.ServiceProxy('/gazebo/pause_physics', Empty)
        #self.reset_proxy = rclpy.ServiceProxy('/gazebo/reset_world', Empty)
        self.action_low = np.array([-1, -1, -1]) # y_vel, x_vel, z_vel
        self.action_high = np.array([1, 1, 1])
        
        self.action_space = spaces.Box(self.action_low, self.action_high, dtype=np.float32)
        self.reward_range = (-np.inf, np.inf)
        self._seed()
        self.radius = 3

        #os.system('python /home/huhaomeng/gym-gazebo/gym_gazebo/envs/px4_uav/mavros_ctrl_server.py &')

        print('@env@ ctrl server started')
        time.sleep(15)

        self.pos = np.array([0, 0, 0])
        #initialize ros nodes
        rclpy.init(args=args)
        #timestamp_subscriber = TimesyncSubscriber()
        vehicle_command_publisher =  VehicleCommandPublisher()
        self.gym_node = GymNode()

        rate = self.gym_node.create_rate(30)
        spin_thread = Thread(target = rclpy.spin, args =(self.gym_node,))
        spin_thread.start()
        offboard_setpoint_counter = 0
        offboard_control_publisher = OffboardControlPublisher()
        print('Nodes initialized')
        #position_publisher = PositionPublisher()
        while(rclpy.ok()):
            timestamp = self.gym_node.current_time
            if offboard_setpoint_counter == 20:
                vehicle_command_publisher.publish(timestamp,176,param1 = 1, param2 =6)
                #arm
                vehicle_command_publisher.publish(timestamp,400,param1 = 1.0)
            #offboard publisher tells px4 which mode to enter I think. You need to do publish_vehicle_command after 10 setpoints
            if offboard_setpoint_counter < 400:
                self.gym_node.x = 0.0
                self.gym_node.y = 0.0
                self.gym_node.z = -5.0
                self.gym_node.yaw =0.0
                self.gym_node.vx = nan
                self.gym_node.vy = nan
                self.gym_node.vz =nan
                self.gym_node.yawspeed = nan
                self.gym_node.mode = 'position'
            #else:
            #    offboard_control_publisher.publish(timestamp, trajectory = True)
            #    timestamp =timestamp_subscriber.current_time
            #    self.gym_node.publish(timestamp, vx = 1.0)
            if offboard_setpoint_counter == 400:
                self.gym_node.x = nan
                self.gym_node.y = nan
                self.gym_node.z = nan
                self.gym_node.yaw =nan
                self.gym_node.vx = 1.0
                self.gym_node.vy = 0.0
                self.gym_node.vz =0.0
                self.gym_node.yawspeed = 0.0
                self.gym_node.mode = 'velocity'
                break
            offboard_setpoint_counter += 1
            #start trajectory publishing thread after desired position is reached

            rate.sleep()


    def step(self, action):
        state = self.gym_node.get_current_state()
        reward = 0
        done = False  # done check
        #start sending new commands
        #TODO it may happen that action zero gets passed first and then send with old action, maybe flag necessary after update to stop that from happening
        self.gym_node.vx = action[0]
        self.gym_node.vy = action[1]
        self.gym_node.vz =action[2]

        # print('@env@ observation:' + str(state))
        # print('@env@ reward:' + str(reward))
        # print('@env@ done:' + str(done))
        return state, reward, done, {'debug': 'working just fine'}

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        print('@env@ Resets the state of the environment and returns an initial observation.')

        rospy.wait_for_service('/gazebo/reset_world')
        try:
            # reset_proxy.call()
            self.reset_proxy()
            # print('@env@ reset model place')
        except rospy.ServiceException as e:
            print ("@env@ /gazebo/reset_world service call failed")

        self.send_msg_get_return('reset')
        print('@env@ sleep 3s')
        time.sleep(3)

        data = self.send_msg_get_return('takeoff')

        data[0] = data[0] - self.des[0]
        data[1] = data[1] - self.des[1]
        data[2] = data[2] - self.des[2]
        self.pos = np.array([0, 0, 0])
        for idx in range(len(data)):
            if idx < 3:
                data[idx] = (data[idx] + 50) / 100
            else:
                if data[idx] > 10 or data[idx] == np.inf:
                    data[idx] = 10
                data[idx] = (data[idx] - 0.2) / 9.8

        state = data
        if 'nan' in str(state):
            state = np.zeros([len(state)])
        return state

    def set_des(self, destination):
        self.des = destination

    def is_at_position(self, tx, ty, tz, x, y, z, offset):
        """offset:meters"""
        desired = np.array((tx, ty, tz))
        pos = np.array((x, y, z))
        return np.linalg.norm(desired - pos) < offset

    def cal_distence(self, old_position, new_position, destination):
        old_distance = np.sqrt(
            np.square(destination[0] - old_position[0]) + np.square(destination[1] - old_position[1]) + np.square(
                destination[2] - old_position[2]))

        new_distance = np.sqrt(
            np.square(destination[0] - new_position[0]) + np.square(destination[1] - new_position[1]) + np.square(
                destination[2] - new_position[2]))

        return old_distance - new_distance

    def stop_ctrl_server(self):
        r_msg = self.send_msg_get_return('shutdown')
        print('@env@ ' + r_msg + ' ctrl_server shutdown')

    def send_msg_get_return(self, msg):
        ctrl_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        while not connected:
            try:
                # print('@env@ try to connect with ctrl server')
                ctrl_client.connect(('localhost', 19881))
                connected = True
                # print('@env@ connected with ctrl server')
            except BaseException as e:
                # print('@env@[Error] ' + str(e))
                time.sleep(1)
                pass

        # done = False
        # while not done:
        # print('@env@ send msg: ' + msg)
        try:
            ctrl_client.send(msg)
            data = pickle.loads(ctrl_client.recv(1024))
            # print('@env@ send msg ' + msg + ' get return: ' + str(data))
            # done = True
        except BaseException as e:
            print ('@env@[Error] ' + str(e))
            time.sleep(1)
        ctrl_client.close()
        return data

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]