import numpy as np
import argparse
import time
import math
from env.multi_main_enviroment import Multi_Drone_Enviroment as ENV

'''
新增多台無人機：https://github.com/mavlink/mavros/issues/88：
'''

# ************************* Design Agent Start ************************* #
class myAgent():
    def __init__(self):
        pass

    def select_action_set(self,obs_set):
        x_dir = 2.5
        y_dir = 0
        z_dir = 0
        pitch = 0
        row = 0
        yaw = 0

        action_set = []
        for i in range(len(obs_set)):
            action = np.array([[x_dir, y_dir, z_dir], [pitch, row, yaw]])
            action_set.append(action)
        return action_set

# ************************* Design Agent Stop  ************************* #

def reset_drone_set(env_set):
    # 注意！ 每一台無人機都會默認自己的起始位置是座標原點[0,0,0]，並不是global frame!!
    init_action = np.array([[0,0,2],[0,0,0*(math.pi/180)]])
    num_env = len(env_set)

    # If Ros is not shut down, pre publish 100 data first
    for i in range(100):   
        checkRosShutdownSet = False
        for i in range(num_env):
            checkRosShutdownSet = checkRosShutdownSet and env_set[i].checkRosShutdown()
        if checkRosShutdownSet:
            break
        else:
            print("Wait for reset setpoint : {}".format(i+1) + " % ", end='\r')
            for i in range(num_env):
                env_set[i].local_pos_pub.publish(env_set[i]._np2PoseStamped(init_action))
            env_set[0].rate.sleep()  # 60 FPS

    print("[State] : Reset & Wait the Drone to the initial point")
    init_time = time.time()
    for i in range(num_env):
        env_set[i].setRosTime()
    while True:
        condition_set = True
        for i in range(num_env):
            C_x = abs(env_set[i].current_pos.pose.position.x - init_action[0][0])
            C_y = abs(env_set[i].current_pos.pose.position.y - init_action[0][1])
            C_z = abs(env_set[i].current_pos.pose.position.z - init_action[0][2])
            condition = (C_x**2+C_y**2+C_z**2)**0.5 < 0.1
            condition_set = condition_set and condition

        if condition_set:
            print("[State] : Initialize Done")
            break
        else:
            for i in range(num_env):
                env_set[i].position_step(init_action)

        current_time = time.time()
        print("Waiting Reset... / FPS : " + "{:.1f}".format(1/(current_time-init_time)), end='\r')
        init_time = current_time
    
    observation_set = []
    for i in range(num_env):
        env_set[i].done = False
        env_set[i].observation.pose = env_set[i]._PoseStamped2np(env_set[i].current_pos)
        env_set[i].observation.img = env_set[i].current_img
        observation_set.append(env_set[i].observation)

    print("[State] : Reset Done")

    return observation_set

def velocity_step_set(env_set, action_set):     
        next_observation_set = []
        done_set = True
        for i in range(len(env_set)):
            next_observation, reward, done, info = env_set[i].velocity_step(action_set[i])
            next_observation_set.append(next_observation)
            done_set = done_set and done

        return next_observation_set, done_set

def shotdown_set(env_set):
    for i in range(len(env_set)):
        env_set[i].shotdown()

def main(args):
    env_set = []
    for i in range(args.drone_num):
        print("Create Drone {}".format(i+1))
        env = ENV("/uav"+str(i+1))
        env_set.append(env)

    agent = myAgent()
    
    for ep in range(2):
        observation_set = reset_drone_set(env_set)

        print("Start Episode : {}".format(ep+1))
        while True:
            action_set = agent.select_action_set(observation_set)
            next_observation_set, done_set = velocity_step_set(env_set, action_set)
            observation_set = next_observation_set
            
            if done_set:
                break
    
    reset_drone_set(env_set)
    shotdown_set(env_set)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone_num", default=3, type=int, help="Total Drone Number")
    args = parser.parse_args()
    main(args)