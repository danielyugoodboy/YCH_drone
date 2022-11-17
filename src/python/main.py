#! /usr/bin/env python

import rospy
import argparse
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

'''
參考文件：https://github.com/danielyugoodboy/NCRL-AIDrone-Platform/tree/master/src
飛行模式：https://docs.px4.io/main/zh/getting_started/flight_modes.html
MAVROS Basics: http://edu.gaitech.hk/gapter/mavros-basics.html
MAVROS_Tutorial: https://masoudir.github.io/mavros_tutorial/

設計不同的飛行模式：


'''

def myhook():
    print("ROS shutdown")
    print("Back to home !")

current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg


def main(args):
    print("State: Start Drone Command")
    rospy.init_node("offb_node_py")

    # ******************** PX4 SETTING ******************** #

    # Set subscriber & publisher
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Set client
    print("State: Wait for Arming service")
    rospy.wait_for_service("/mavros/cmd/arming")  # 等待 service 可用
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    print("State: Wait for Set_mode service")
    rospy.wait_for_service("/mavros/set_mode")  # 等待 service 可用
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    print("State: Wait for Connection")
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    print("State: Connected !")

    # Define inital position
    pose = PoseStamped()
    pose.pose.position.x = 2
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(50):   
        if(rospy.is_shutdown()):
            break
        else:
            print("Wait for setpoint : " + "{}".format((i+1)*2) + " % ", end='\r')
            local_pos_pub.publish(pose)
            rate.sleep()

    print("State: Set SetModeRequest")
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    land_set_mode = SetModeRequest()
    land_set_mode.custom_mode = 'LAND'

    print("State: Set CommandBoolRequest")
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    print("State: Start")
    try:
        while(not rospy.is_shutdown()):
            if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()

            else:
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            local_pos_pub.publish(pose)

            rate.sleep()

    except KeyboardInterrupt:
        print('KeyboardInterrupt!')

    finally:
        set_mode_client.call(land_set_mode).mode_sent
        rospy.on_shutdown(myhook)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default=1.0, type=float, help="Just For Test")
    args = parser.parse_args()
    main(args)