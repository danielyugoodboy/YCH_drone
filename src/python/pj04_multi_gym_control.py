import numpy as np
import argparse
import time
import math
from env.multi_main_enviroment import Multi_Drone_Enviroment as ENV

'''
To add a third iris to this simulation there are two main components to consider:

* 把UAV3 添加到multi_uav_mavros_sitl.launch
    *复制已经存在的四旋翼(UAV1 或者 UAV2)
    * 把 ID 改为 3
    * 与gazebo的通信，选择一个不同的 mavlink_udp_port端口
    * MAVROS通信端口选择是通过在fcu_url 中修改两个端口号。
* 创建一个开始文件，并按照如下方式修改：
    * 复制已存在的iris rcs启动文件，(iris_1 或 iris_2) ，重命名为iris_3
    * MAV_SYS_ID 值改为3
    * SITL_UDP_PRT 的值与 mavlink_udp_port相一致。
    * 第一个mavlink start 端口和mavlink stream端口值设置为相同值，用于和QGC通信。
    * 第二个mavlink start 端口值应与启动文件 fcu_url 中的值一致。
'''

# ************************* Design Agent Start ************************* #
class myAgent():
    def __init__(self):
        pass

    def select_action_set(self,obs_set):
        x_dir = 2.5
        y_dir = 0
        z_dir = 0.5
        pitch = 0
        row = 0
        yaw = 0.2

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
    for j in range(100):   
        checkRosShutdownSet = False
        for i in range(num_env):
            checkRosShutdownSet = checkRosShutdownSet or env_set[i].checkRosShutdown()
        if checkRosShutdownSet:
            break
        else:
            for i in range(num_env):
                env_set[i].local_pos_pub.publish(env_set[i]._np2PoseStamped(init_action))
        print("Wait for reset setpoint : {}".format(j+1) + " % ", end='\r')
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

def vel_PID_control(env_set, diff_pos_set):
    pass

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
            #**************** Virtual Drone Design ****************#
            '''
            1. 虛擬無人機為 golbal frame 此資料來自 所有實體無人機的型心在進行微調 
            2. 虛擬無人機主要是以位置與姿態控制，而非速度控制，這樣比較好保持隊形整齊
            3. 也就是輸入值為虛擬無人機期望到達的位置與姿態
            4. 控制主要在body frame當中控制
            5. 也就是回傳的值是body frame的僚機座標
            '''





            #******************************************************#  

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