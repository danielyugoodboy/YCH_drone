#! /usr/bin/env python

import rospy
import argparse
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

CONMAND_FPS = 25

'''
Ref：https://github.com/danielyugoodboy/NCRL-AIDrone-Platform/tree/master/src
Flying Mode：https://docs.px4.io/main/zh/getting_started/flight_modes.html
MAVROS Basics: http://edu.gaitech.hk/gapter/mavros-basics.html
MAVROS_Tutorial: https://masoudir.github.io/mavros_tutorial/

https://github.com/mavlink/mavros/issues/1158

Design different flying mode：
1.RPYT
2.Position

'''

current_state_1 = State()
current_pos_1 = PoseStamped()
current_state_2 = State()
current_pos_2 = PoseStamped()

def myhook():
    print("ROS shutdown")
    print("Back to home !")

def state_cb_1(msg):
    global current_state_1
    current_state_1 = msg

def pos_cb_1(msg):
    global current_pos_1
    current_pos_1 = msg

def state_cb_2(msg):
    global current_state_2
    current_state_2 = msg

def pos_cb_2(msg):
    global current_pos_2
    current_pos_2 = msg

def main(args):
    print("State: Start Drone Command")
    rospy.init_node("ori_ROS_demo")

    # ************************* PX4 SETTING ************************* #

    # [Drone 1] Set subscriber & publisher
    state_sub_1 = rospy.Subscriber("/uav1/mavros/state", State, callback = state_cb_1)
    local_pos_sub_1 = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, callback = pos_cb_1)
    local_pos_pub_1 = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # [Drone 1] Set client
    print("State: Wait for Arming service")
    rospy.wait_for_service("/uav1/mavros/cmd/arming")  # wait service
    arming_client_1 = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)

    print("State: Wait for Set_mode service")
    rospy.wait_for_service("/uav1/mavros/set_mode")  # wait service
    set_mode_client_1 = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)

    # [Drone 2] Set subscriber & publisher
    state_sub_2 = rospy.Subscriber("/uav2/mavros/state", State, callback = state_cb_2)
    local_pos_sub_2 = rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, callback = pos_cb_2)
    local_pos_pub_2 = rospy.Publisher("/uav2/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # [Drone 2] Set client
    print("State: Wait for Arming service")
    rospy.wait_for_service("/uav2/mavros/cmd/arming")  # wait service
    arming_client_2 = rospy.ServiceProxy("/uav2/mavros/cmd/arming", CommandBool)

    print("State: Wait for Set_mode service")
    rospy.wait_for_service("/uav2/mavros/set_mode")  # wait service
    set_mode_client_2 = rospy.ServiceProxy("/uav2/mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(CONMAND_FPS)

    # Wait for Flight Controller connection
    print("State: Wait for Connection")
    while(not rospy.is_shutdown() and not (current_state_1.connected and current_state_2.connected)):
        rate.sleep()

    print("State: Set SetModeRequest")
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    print("State: Set CommandBoolRequest")
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    # ************************* MAIN FUCTION ************************ #

    try:
        # Define inital position
        pose_1 = PoseStamped()
        pose_1.pose.position.x = 0
        pose_1.pose.position.y = 0
        pose_1.pose.position.z = 1

        pose_2 = PoseStamped()
        pose_2.pose.position.x = 2
        pose_2.pose.position.y = 0
        pose_2.pose.position.z = 1

        # Publish inital point before start
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            else:
                print("Wait for setpoint : " + "{}".format(i+1) + " % ", end='\r')
                local_pos_pub_1.publish(pose_1)
                local_pos_pub_2.publish(pose_2)
                rate.sleep()
        
        
        print("State: Start")
        last_req_1 = rospy.Time.now()
        last_req_2 = rospy.Time.now()
        init_time = time.time()
        while(not rospy.is_shutdown()):
            # [Drone 1] Confirm current_state.mode == "OFFBOARD"，give 5 sec to wait
            if(current_state_1.mode != "OFFBOARD" and (rospy.Time.now() - last_req_1) > rospy.Duration(5.0)):
                if(set_mode_client_1.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req_1 = rospy.Time.now()

            # [Drone 1] Confirm current_state.armed == True，give 5 sec to wait
            else:
                if(not current_state_1.armed and (rospy.Time.now() - last_req_1) > rospy.Duration(5.0)):
                    if(arming_client_1.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req_1 = rospy.Time.now()
            
            # [Drone 2] Confirm current_state.mode == "OFFBOARD"，give 5 sec to wait
            if(current_state_2.mode != "OFFBOARD" and (rospy.Time.now() - last_req_2) > rospy.Duration(5.0)):
                if(set_mode_client_2.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req_2 = rospy.Time.now()

            # [Drone 2] Confirm current_state.armed == True，give 5 sec to wait
            else:
                if(not current_state_2.armed and (rospy.Time.now() - last_req_2) > rospy.Duration(5.0)):
                    if(arming_client_2.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req_2 = rospy.Time.now()

            # Publish pose
            local_pos_pub_1.publish(pose_1)
            pose_1.pose.position.x += 0.005
            local_pos_pub_2.publish(pose_2)
            pose_2.pose.position.x += 0.005

            # calculate FPS
            current_time = time.time()
            print("Control Conmand FPS : " + "{:.1f}".format(1/(current_time-init_time)), end='\r')
            init_time = current_time
            rate.sleep()

    except KeyboardInterrupt:
        print('Keyboard Interrupt!')

    finally:
        rospy.on_shutdown(myhook)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default=1.0, type=float, help="Just For Test")
    args = parser.parse_args()
    main(args)