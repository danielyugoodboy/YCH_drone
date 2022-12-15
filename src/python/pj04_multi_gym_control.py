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
        self.init_time = time.time()

    def select_action_set(self,env_set, vir_drone, vir_drone_pos):
        # Movement design
        print(vir_drone_pos)
        #target_pos = vir_drone_pos + np.array([[0,0,0],[0,0,0.1]])
        target_pos = np.array([[-3,-3,3],[0,0,0]])

        # Calculate Error
        body_pos_set = vir_drone.cal_body_frame(target_pos, env_set)
        
        error_set = []
        for i in range(len(body_pos_set)):
            bC_x = vir_drone.formation_1[i][0][0] - body_pos_set[i][0][0]
            bC_y = vir_drone.formation_1[i][0][1] - body_pos_set[i][0][1]
            # target point rotate (以無人機視角看目標點)
            C_x = math.cos(-body_pos_set[i][1][2])*bC_x - math.sin(-body_pos_set[i][1][2])*bC_y
            C_y = math.sin(-body_pos_set[i][1][2])*bC_x + math.cos(-body_pos_set[i][1][2])*bC_y
            C_z = vir_drone.formation_1[i][0][2] - body_pos_set[i][0][2] 
            C_yaw = normalize_angle(vir_drone.formation_1[i][1][2]-body_pos_set[i][1][2])
            error_set.append(np.array([[C_x, C_y, C_z], [0,0,C_yaw]]))

        # PID Controller
        current_time = time.time()
        dt = current_time-self.init_time
        control_input_set = vir_drone.vel_PID_control(error_set, dt)
        self.init_time = current_time

        return control_input_set

    def reset_time(self):
        self.init_time = time.time()

# ************************* Design Agent Stop  ************************* #

class Virtual_Drone():
    def __init__(self, drone_num):
        '''
        1. 虛擬無人機為 golbal frame 此資料來自 所有實體無人機的型心在進行微調 
        2. 虛擬無人機主要是以位置與姿態控制，而非速度控制，這樣比較好保持隊形整齊
        3. 也就是輸入值為虛擬無人機期望到達的位置與姿態
        4. 控制主要在body frame當中控制
        5. 也就是回傳的值是body frame的僚機座標

        formation_1:
        X X 4 X X
        X 2 X 3 X
        X X 1 X X
        '''
        self.drone_num = drone_num
        self.formation_1 = np.array([[[-2,0,0],[0,0,0]],
                                     [[2,2,0],[0,0,0]],
                                     [[2,-2,0],[0,0,0]],
                                     [[2,0,0],[0,0,0]]])
        self.error_sum_set = []
        self.error_past_set = []
        for i in range(drone_num):
            error_sum = np.array([[0.,0.,0.],[0.,0.,0.]])
            error_past = np.array([[0.,0.,0.],[0.,0.,0.]])
            self.error_sum_set.append(error_sum)
            self.error_past_set.append(error_past)

    def cal_current_pos(self, env_set):
        drone_num = len(env_set)
        pos_X, pos_Y, pos_Z, pos_Yaw = 0,0,0,0
        for i in range(drone_num):
            pos_X += env_set[i].current_global_pos[0]
            pos_Y += env_set[i].current_global_pos[1]
            pos_Z += env_set[i].current_global_pos[2]
            pos_Yaw += env_set[i].observation.local_pose[1][2]
        
        pos_Yaw = normalize_angle(pos_Yaw/drone_num)
        global_pos = np.array([[pos_X/drone_num, pos_Y/drone_num, pos_Z/drone_num],[0,0,pos_Yaw]])

        return global_pos

    def cal_body_frame(self, tar_global_pos, env_set):
        drone_num = len(env_set)
        global_yaw = tar_global_pos[1][2]
        body_pos_set = []
        for i in range(drone_num):
            diff_pos = env_set[i].current_global_pos-tar_global_pos[0]
            diff_yaw = env_set[i]._PoseStamped2np(env_set[i].current_pos)[1][2]-tar_global_pos[1][2]
            diff_yaw = normalize_angle(diff_yaw)

            body_pos_x = math.cos(-global_yaw)*diff_pos[0] - math.sin(-global_yaw)*diff_pos[1]
            body_pos_y = math.sin(-global_yaw)*diff_pos[0] + math.cos(-global_yaw)*diff_pos[1]
            body_pos = np.array([[body_pos_x, body_pos_y, diff_pos[2]],[0,0,diff_yaw]])
            body_pos_set.append(body_pos)

        return body_pos_set

    def vel_PID_control(self, error_set, dt):
        control_input_set = []
        kp_linear, ki_linear, kd_linear = 3.0, 0.5*(1.1**5), 0.15*(1.1**5)
        kp_rotate, ki_rotate, kd_rotate = 1.0, 0.0, 0.0
        for i in range(self.drone_num):
            # X-dir
            e_x = error_set[i][0][0]

            self.error_sum_set[i][0][0] = self.error_sum_set[i][0][0]*0.9 + e_x*dt
            e_x_sum = self.error_sum_set[i][0][0]
            e_x_past = self.error_past_set[i][0][0]

            input_X = kp_linear*e_x + ki_linear*e_x_sum + kd_linear*((e_x-e_x_past)/dt)
            input_X = max(min(input_X,10),-10)
            self.error_past_set[i][0][0] = e_x
            
            # Y-dir
            e_y = error_set[i][0][1]
            
            self.error_sum_set[i][0][1] = self.error_sum_set[i][0][1]*0.9 + e_y*dt
            e_y_sum = self.error_sum_set[i][0][1]
            e_y_past = self.error_past_set[i][0][1]
            
            input_Y = kp_linear*e_y + ki_linear*e_y_sum + kd_linear*((e_y-e_y_past)/dt)
            input_Y = max(min(input_Y,10),-10)
            self.error_past_set[i][0][1] = e_y
            
            # Z-dir
            e_z = error_set[i][0][2]
            
            self.error_sum_set[i][0][2] = self.error_sum_set[i][0][2]*0.9 + e_z*dt
            e_z_sum = self.error_sum_set[i][0][2]
            e_z_past = self.error_past_set[i][0][2]
            
            input_Z = kp_linear*e_z + ki_linear*e_z_sum + kd_linear*((e_z-e_z_past)/dt)
            input_Z = max(min(input_Z,10),-10)
            self.error_past_set[i][0][2] = e_z
            
            # Yaw-dir
            e_yaw = error_set[i][1][2]
            
            self.error_sum_set[i][1][2] = self.error_sum_set[i][1][2]*0.9 + e_yaw*dt
            e_yaw_sum = self.error_sum_set[i][1][2]
            e_yaw_past = self.error_past_set[i][1][2]
            
            input_Yaw = kp_rotate*e_yaw + ki_rotate*e_yaw_sum + kd_rotate*((e_yaw-e_yaw_past)/dt)
            input_Yaw = max(min(input_Yaw,2),-2)
            self.error_past_set[i][1][2] = e_yaw

            control_input_set.append(np.array([[input_X, input_Y, input_Z],[0,0,input_Yaw]]))
        
        return control_input_set
    
    def reset_PID_control(self):
        for i in range(self.drone_num):
            error_sum = np.array([[0.,0.,0.],[0.,0.,0.]])
            error_past = np.array([[0.,0.,0.],[0.,0.,0.]])
            self.error_sum_set[i] = error_sum
            self.error_past_set[i] = error_past

def reset_drone_set(env_set, vir_drone, init_position):
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
                env_set[i].local_pos_pub.publish(env_set[i]._np2PoseStamped(init_position))
        print("Wait for reset setpoint : {}".format(j+1) + " % ", end='\r')
        env_set[0].rate.sleep()  # 60 FPS

    print("[State] : Reset & Wait the Drone to the initial point")
    init_time = time.time()
    for i in range(num_env):
        env_set[i].setRosTime()

    # Go Fly
    while True:
        # Calculate Error
        body_pos_set = vir_drone.cal_body_frame(init_position, env_set)        
        condition_set = True
        error_set = []
        for i in range(num_env):
            bC_x = vir_drone.formation_1[i][0][0] - body_pos_set[i][0][0]
            bC_y = vir_drone.formation_1[i][0][1] - body_pos_set[i][0][1]
            # target point rotate (以無人機視角看目標點)
            C_x = math.cos(-body_pos_set[i][1][2])*bC_x - math.sin(-body_pos_set[i][1][2])*bC_y
            C_y = math.sin(-body_pos_set[i][1][2])*bC_x + math.cos(-body_pos_set[i][1][2])*bC_y
            C_z = vir_drone.formation_1[i][0][2] - body_pos_set[i][0][2] 
            C_yaw = normalize_angle(vir_drone.formation_1[i][1][2]-body_pos_set[i][1][2])
            error_set.append(np.array([[C_x, C_y, C_z], [0,0,C_yaw]]))
            
            condition = (C_x**2+C_y**2+C_z**2)**0.5 < 0.05 and abs(C_yaw)< 0.035  # 2 degree
            condition_set = condition_set and condition
        
        # PID Controller
        current_time = time.time()
        dt = current_time-init_time
        if condition_set:
            print("[State] : Initialize Done")
            vir_drone.reset_PID_control
            break
        else:
            control_input_set = vir_drone.vel_PID_control(error_set, dt)
            for i in range(num_env):
                env_set[i].velocity_step(control_input_set[i])

        print("Waiting Reset... / FPS : " + "{:.1f}".format(1/(dt)), end='\r')
        init_time = current_time
    
    # Record Observation
    observation_set = []
    for i in range(num_env):
        env_set[i].done = False
        env_set[i].observation.local_pose = env_set[i]._PoseStamped2np(env_set[i].current_pos)
        env_set[i].observation.img = env_set[i].current_img
        observation_set.append(env_set[i].observation)
    

    vir_drone_pos = vir_drone.cal_current_pos(env_set)
    print("[State] : Reset Done")
    return vir_drone_pos

def velocity_step_set(env_set, vir_drone, action_set):     
        next_observation_set = []
        done_set = True
        for i in range(len(env_set)):
            next_observation, reward, done, info = env_set[i].velocity_step(action_set[i])
            next_observation_set.append(next_observation)
            done_set = done_set and done
        next_vir_drone_pos = vir_drone.cal_current_pos(env_set)
        return next_vir_drone_pos, done_set

def shotdown_set(env_set):
    for i in range(len(env_set)):
        env_set[i].shotdown()

def normalize_angle(angle):
    angle = (angle+2*math.pi)%(2*math.pi)
    if angle > math.pi:
        angle -= 2*math.pi
    return angle

def main(args):
    env_set = []
    for i in range(args.drone_num):
        print("Create Drone {}".format(i+1))
        env = ENV("/uav"+str(i+1))
        env_set.append(env)

    vir_drone = Virtual_Drone(args.drone_num)
    agent = myAgent()
    
    for ep in range(1):
        init_position = np.array([[-3,-3,3],[0,0,math.pi/4]])
        vir_drone_pos = reset_drone_set(env_set, vir_drone, init_position)
        agent.reset_time()

        print("Start Episode : {}".format(ep+1))
        while True: 
            action_set = agent.select_action_set(env_set, vir_drone, vir_drone_pos)
            next_vir_drone_pos, done_set = velocity_step_set(env_set, vir_drone, action_set)
            vir_drone_pos = next_vir_drone_pos
            
            if done_set:
                break
    
    final_position = np.array([[0,0,1.0],[0,0,0]])
    reset_drone_set(env_set, vir_drone, final_position)
    shotdown_set(env_set)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone_num", default=3, type=int, help="Total Drone Number")
    args = parser.parse_args()
    main(args)