import numpy as np
import argparse
import time
import math
from env.multi_main_enviroment import Multi_Drone_Enviroment as ENV

# ************************* Design Agent Start ************************* #
class myAgent():
    def __init__(self):
        pass

    def select_action(self,obs):
        x_dir = 1
        y_dir = 0
        z_dir = 0
        pitch = 0
        row = 0
        yaw = 0

        action = np.array([[x_dir, y_dir, z_dir], [pitch, row, yaw]])
        return action

# ************************* Design Agent Stop  ************************* #
'''
1. action :
numpy array
shape = (2, 3)
np.array([x,y,z],[pitch, roll, yaw])

2. observation.pose :
numpy array
shape = (2, 3)
np.array([x,y,z],[pitch, roll, yaw])

3. observation.img :
numpy array
shape = (240, 320, 3)
'''

def reset_drone(env_1, env_2):
    init_action_1 = np.array([[0,0,2],[0,0,0*(math.pi/180)]])
    init_action_2 = np.array([[2,0,2],[0,0,0*(math.pi/180)]])

    for i in range(100):   
        if(env_1.checkRosShutdown() or env_2.checkRosShutdown()):
            break
        else:
            print("Wait for reset setpoint : {}".format(i+1) + " % ", end='\r')
            env_1.local_pos_pub.publish(env_1._np2PoseStamped(init_action_1))
            env_2.local_pos_pub.publish(env_2._np2PoseStamped(init_action_2))
            env_1.rate.sleep()

    print("[State] : Reset & Wait the Drone to the initial point")
    init_time = time.time()
    env_1.setRosTime()
    env_2.setRosTime()
    while True:
        C_1_1 = abs(env_1.current_pos.pose.position.x - init_action_1[0][0]) < 0.1
        C_1_2 = abs(env_1.current_pos.pose.position.y - init_action_1[0][1]) < 0.1
        C_1_3 = abs(env_1.current_pos.pose.position.z - init_action_1[0][2]) < 0.1
        C_1 = C_1_1 and C_1_2 and C_1_3

        C_2_1 = abs(env_2.current_pos.pose.position.x - init_action_2[0][0]) < 0.1
        C_2_2 = abs(env_2.current_pos.pose.position.y - init_action_2[0][1]) < 0.1
        C_2_3 = abs(env_2.current_pos.pose.position.z - init_action_2[0][2]) < 0.1
        C_2 = C_2_1 and C_2_2 and C_2_3

        if C_1 and C_2:
            print("[State] : Initialize Done")
            break
        else:
            env_1.position_step(init_action_1)
            env_2.position_step(init_action_2)

        current_time = time.time()
        print("Waiting Reset... / FPS : " + "{:.1f}".format(1/(current_time-init_time)), end='\r')
        init_time = current_time
    
    env_1.done = False
    env_2.done = False
    print("[State] : Reset Done")

    env_1.observation.pose = env_1._PoseStamped2np(env_1.current_pos)
    env_1.observation.img = env_1.current_img
    env_2.observation.pose = env_2._PoseStamped2np(env_2.current_pos)
    env_2.observation.img = env_2.current_img

    return [env_1.observation, env_2.observation]

def main(args):
    env_1 = ENV("/uav1")
    env_2 = ENV("/uav2")

    agent = myAgent()
    
    for ep in range(2):
        observation_set = reset_drone(env_1, env_2)
        observation_1 = observation_set[0]
        observation_2 = observation_set[1]

        print("Start Episode : {}".format(ep+1))
        while True:
            action_1 = agent.select_action(observation_1)
            action_2 = agent.select_action(observation_2)
            next_observation_1, reward_1, done_1, info_1 = env_1.velocity_step(action_1)
            next_observation_2, reward_2, done_2, info_2 = env_2.velocity_step(action_2)
            observation_1 = next_observation_1
            observation_2 = next_observation_2
            
            if done_1 and done_2:
                break
    
    reset_drone(env_1, env_2)
    
    env_1.shotdown()
    env_2.shotdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default=1.0, type=float, help="Just For Test")
    args = parser.parse_args()
    main(args)