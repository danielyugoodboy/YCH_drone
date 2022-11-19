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

current_state = State()
current_pos = PoseStamped()

def myhook():
    print("ROS shutdown")
    print("Back to home !")

def state_cb(msg):
    global current_state
    current_state = msg

def pos_cb(msg):
    global current_pos
    current_pos = msg

def main(args):
    print("State: Start Drone Command")
    rospy.init_node("offb_node_py")

    # ************************* PX4 SETTING ************************* #

    # Set subscriber & publisher
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback = pos_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Set client
    print("State: Wait for Arming service")
    rospy.wait_for_service("/mavros/cmd/arming")  # wait service
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    print("State: Wait for Set_mode service")
    rospy.wait_for_service("/mavros/set_mode")  # wait service
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(CONMAND_FPS)

    # Wait for Flight Controller connection
    print("State: Wait for Connection")
    while(not rospy.is_shutdown() and not current_state.connected):
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
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 1

        # Publish inital point before start
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            else:
                print("Wait for setpoint : " + "{}".format(i+1) + " % ", end='\r')
                local_pos_pub.publish(pose)
                rate.sleep()
        
        
        print("State: Start")
        last_req = rospy.Time.now()
        init_time = time.time()
        while(not rospy.is_shutdown()):
            # Confirm current_state.mode == "OFFBOARD"，give 5 sec to wait
            if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()

            # Confirm current_state.armed == True，give 5 sec to wait
            else:
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            # Get current_state
            #print(current_pos.pose.position)
            #print(current_pos.pose.orientation)

            # Publish pose
            local_pos_pub.publish(pose)
            pose.pose.position.x += 0.005
            pose.pose.orientation.z += 0.005

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