#! /usr/bin/env python

import rospy
import argparse
import time
import math
import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import Imu, Image, NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

CONMAND_FPS = 60
ORI_LAT = 47.3977419  # 緯度
ORI_LON = 8.5455936  # 經度
ORI_ALT = 535.2399161  # 高度
EARTH_R = 6371000  # 地球半徑(m)

'''
Ref：https://github.com/danielyugoodboy/NCRL-AIDrone-Platform/tree/master/src
Flying Mode：https://docs.px4.io/main/zh/getting_started/flight_modes.html
MAVROS Basics: http://edu.gaitech.hk/gapter/mavros-basics.html
MAVROS_Tutorial: https://masoudir.github.io/mavros_tutorial/
'''

class Drone_observation():
    def __init__(self):
        '''
        0. action :
        * control input 可以是位置姿態，也可以是速度角速度
        numpy array
        shape = (2, 3)
        np.array([[x,y,z],[pitch, roll, yaw]])

        1. observation.local_pose :
        * 以出生點為座標原點
        numpy array
        shape = (2, 3)
        np.array([[x,y,z],[pitch, roll, yaw]])

        2. observation.global_pose :
        * GPS 座標
        numpy array
        shape = (3,)
        np.array([x,y,z])

        3. observation.img :
        * 影像資訊 目前並沒有使用
        numpy array
        shape = (240, 320, 3)
        '''
        self.local_pose = None
        self.global_pose = None
        self.img = None

class Drone_Enviroment():
    def __init__(self, drone_name=""):
        print("[State] : Start Drone{} Enviroment".format(drone_name))
        rospy.init_node("main_enviroemnt")

        # *********************** INITIAL PARAMETER ********************* #
        # Initial parameter setup
        self.last_req = rospy.Time.now()
        self.observation = Drone_observation()

        self.current_state = State()
        self.current_local_pos = PoseStamped()  # For orientation & height
        self.current_global_pos = np.array([0,0,0])  # For position
        self.current_img = Image()
        
        self.altitude = 0.0
        
        self.done = False
        self.name = drone_name
        self.distant_limit = 15.0

        # ************************* PX4 SETTING ************************* #

        # Set subscriber
        state_sub = rospy.Subscriber(self.name+"/mavros/state", State, callback = self.state_cb)
        local_pos_sub = rospy.Subscriber(self.name+"/mavros/local_position/pose", PoseStamped, callback = self.pos_cb)
        gps_sub = rospy.Subscriber(self.name+"/mavros/global_position/global", NavSatFix, callback = self.gps_cb)
        #local_img_sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, callback = self.img_cb)

        # Set publisher
        self.local_pos_pub = rospy.Publisher(self.name+"/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpoint_velocity_pub = rospy.Publisher(self.name+"/mavros/setpoint_velocity/cmd_vel",TwistStamped, queue_size = 10)

        # Set client
        print("[State] : Wait for Arming service")
        rospy.wait_for_service(self.name+"/mavros/cmd/arming")  # wait service
        self.arming_client = rospy.ServiceProxy(self.name+"/mavros/cmd/arming", CommandBool)

        print("[State] : Wait for Set_mode service")
        rospy.wait_for_service(self.name+"/mavros/set_mode")  # wait service
        self.set_mode_client = rospy.ServiceProxy(self.name+"/mavros/set_mode", SetMode)

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(CONMAND_FPS)

        # Wait for Flight Controller connection
        print("[State] : Wait for Connection")
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

        print("[State] : Set SetModeRequest")
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        print("[State] : Set CommandBoolRequest")
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

    def state_cb(self, data):
        self.current_state = data

    def pos_cb(self, data):
        self.current_local_pos = data
        self.altitude = data.pose.position.z

    def gps_cb(self, data):
        raw_gps = data

        diff_lat = raw_gps.latitude-ORI_LAT
        diff_lon = raw_gps.longitude-ORI_LON

        global_X = math.pi*(EARTH_R*math.cos(ORI_LAT*(math.pi/180)))*(diff_lon/180)
        global_Y = math.pi*EARTH_R*(diff_lat/180)
        global_Z = self.altitude

        self.current_global_pos = np.array([global_X, global_Y, global_Z])

    def img_cb(self, data):
        image = ros_numpy.numpify(data).copy()
        self.current_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    def position_step(self, action):
        # Confirm current_state.mode == "OFFBOARD"，give 5 sec to wait
        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            self.setRosTime()

        # Confirm current_state.armed == True，give 5 sec to wait
        else:
            if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                if(self.arming_client.call(self.arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                self.setRosTime()

        # 1. Publish action
        self.local_pos_pub.publish(self._np2PoseStamped(action))

        # 2. Get observation
        self.observation.local_pose = self._PoseStamped2np(self.current_local_pos)
        self.observation.global_pose = self.current_global_pos
        self.observation.img = self.current_img

        # 3. Calculate reward (for reinforcement learning)
        reward = 0

        # 4. Done or not
        linear_distant = np.sum(np.square(self.observation.local_pose[0]))**0.5
        if linear_distant >= self.distant_limit:
            self.done = True

        # 5. Info
        info = "Nothing"

        self.rate.sleep()
        return self.observation, reward, self.done, info

    def velocity_step(self, action):
        # Confirm current_state.mode == "OFFBOARD"，give 5 sec to wait
        if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
            if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            self.setRosTime()

        # Confirm current_state.armed == True，give 5 sec to wait
        else:
            if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                if(self.arming_client.call(self.arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                self.setRosTime()
        
        # 1. Publish action
        set_velocity = TwistStamped()
        yaw = self._PoseStamped2np(self.current_local_pos)[1][2]
        set_velocity.twist.linear.x = action[0][0]*math.cos(yaw)-action[0][1]*math.sin(yaw)
        set_velocity.twist.linear.y = action[0][0]*math.sin(yaw)+action[0][1]*math.cos(yaw)
        set_velocity.twist.linear.z = action[0][2]
        set_velocity.twist.angular.z = action[1][2]
        self.setpoint_velocity_pub.publish(set_velocity)

        # 2. Get observation
        self.observation.local_pose = self._PoseStamped2np(self.current_local_pos)
        self.observation.global_pose = self.current_global_pos
        self.observation.img = self.current_img

        # 3. Calculate reward (for reinforcement learning)
        reward = 0

        # 4. Done or not
        linear_distant = np.sum(np.square(self.observation.local_pose[0]))**0.5
        if linear_distant >= self.distant_limit:
            self.done = True
        
        # 5. Info
        info = "Nothing"

        self.rate.sleep()
        return self.observation, reward, self.done, info

    def reset(self):
        # ************************ FLY TO INITIAL *********************** #
        # Publish inital point before start
        init_action = np.array([[0,0,2],[0,0,0*(math.pi/180)]])  # [x, y, z],[pitch, roll, yaw]
        
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            else:
                print("Wait for reset setpoint : {}".format(i+1) + " % ", end='\r')
                self.local_pos_pub.publish(self._np2PoseStamped(init_action))
                self.rate.sleep()
        
        # Wait the Drone to the initial point
        print("[State] : Reset & Wait the Drone to the initial point")
        init_time = time.time()
        self.setRosTime()
        while(not rospy.is_shutdown()):
            C_1 = abs(self.current_local_pos.pose.position.x - init_action[0][0]) < 0.2
            C_2 = abs(self.current_local_pos.pose.position.y - init_action[0][1]) < 0.2
            C_3 = abs(self.current_local_pos.pose.position.z - init_action[0][2]) < 0.2

            if C_1 and C_2 and C_3:  # 1cm
                print("[State] : Initialize Done")
                break
            else:
                self.position_step(init_action)
        
            # calculate FPS
            current_time = time.time()
            print("Waiting Reset... / FPS : " + "{:.1f}".format(1/(current_time-init_time)), end='\r')
            init_time = current_time
        
        self.done = False
        print("[State] : Reset Done")

        self.observation.local_pose = self._PoseStamped2np(self.current_local_pos)
        self.observation.img = self.current_img

        return self.observation

    def shotdown(self):
        rospy.on_shutdown(self._myhook)

    def _np2PoseStamped(self, np_action):
        '''
        Eularian to Quaternion
        這邊的姿態使用的是四元數（Quaternion）
        q = cos(theta/2)+sin(theta/2)n
        q = cos(theta/2)+sin(theta/2)i+sin(theta/2)j+sin(theta/2)k
        q = w + xi + yj + zk
        '''
        pose = PoseStamped()
        pose.pose.position.x = np_action[0][0]
        pose.pose.position.y = np_action[0][1]
        pose.pose.position.z = np_action[0][2]
        pose.pose.orientation.x = 0*math.sin(np_action[1][2]/2)
        pose.pose.orientation.y = 0*math.sin(np_action[1][2]/2)
        pose.pose.orientation.z = 1*math.sin(np_action[1][2]/2)
        pose.pose.orientation.w = math.cos(np_action[1][2]/2)

        return pose

    def _PoseStamped2np(self, pose):
        '''
        Quaternion to Eularian
        這邊的姿態使用的是四元數（Quaternion）
        q = cos(theta/2)+sin(theta/2)n
        q = cos(theta/2)+sin(theta/2)i+sin(theta/2)j+sin(theta/2)k
        q = w + xi + yj + zk
        '''
        pos_X = pose.pose.position.x
        pos_Y = pose.pose.position.y
        pos_Z = pose.pose.position.z
        
        q_w = pose.pose.orientation.w
        q_x = pose.pose.orientation.x
        q_y = pose.pose.orientation.y
        q_z = pose.pose.orientation.z

        # roll
        #roll = math.atan2(2*(q_w*q_x + q_y*q_z), 1-2*(q_x**2 + q_y**2))
        
        # pitch
        #pitch  = math.asin(np.clip(2*(q_w*q_y - q_z*q_z), -1.0, 1.0))

        # yaw
        yaw = math.atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_y**2 + q_z**2))

        return np.array([[pos_X,pos_Y,pos_Z],[0, 0, yaw]])
    
    def setRosTime(self):
        self.last_req = rospy.Time.now()
    
    def checkRosShutdown(self):
        if rospy.is_shutdown():
            return True
        else:
            return False

    def _myhook(self):
        print("[State] : ROS shutdown, Drone Back to home !")


def test_main():
    env = Drone_Enviroment("/uav1")
    observation = env.reset()
    action = np.array([[2,0,0],[0,0,0.2]])
    while True:
        env.velocity_step(action)
        if env.done:
            break 
    env.reset()
    env.shotdown()

if __name__ == "__main__":
    test_main()