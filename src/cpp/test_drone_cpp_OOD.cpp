#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
using namespace std;

class Drone_Enviroment {
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Publisher local_pos_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped pose_;

    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

public:
    string drone_name;
    Drone_Enviroment(string drone_name):drone_name(drone_name) {
        state_sub_ = nh_.subscribe<mavros_msgs::State>(drone_name+"mavros/state", 10, &Drone_Enviroment::stateCb, this);
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(drone_name+"mavros/setpoint_position/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(drone_name+"mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(drone_name+"mavros/set_mode");

        pose_.pose.position.x = 0;
        pose_.pose.position.y = 0;
        pose_.pose.position.z = 2;
    }

    void run() {
        ros::Rate rate(20.0);

        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        ros::Time last_request = ros::Time::now();

        while (ros::ok()) {
            if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                mavros_msgs::SetMode offb_set_mode;
                offb_set_mode.request.custom_mode = "OFFBOARD";
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    mavros_msgs::CommandBool arm_cmd;
                    arm_cmd.request.value = true;
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }

            local_pos_pub_.publish(pose_);

            ros::spinOnce();
            rate.sleep();
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    Drone_Enviroment env("uav1/");
    env.run();
    return 0;
}