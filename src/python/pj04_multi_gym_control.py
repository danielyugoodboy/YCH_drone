import numpy as np
import argparse
import time
import math
from env.main_enviroment import Drone_Enviroment as ENV

'''
To add a third iris to this simulation there are two main components to consider:

* Step_1. Add UAV3 to "/YCH_drone/launch/uav_mavros_sitl_multiDrone.launch"
    * Copy an existing quadcopter (UAV1 or UAV2)
    * Change ID to '3'
    * Change 2 different port of "fcu_url"  (Link mavros)
    * Change a different "mavlink_udp_port"  (Link Gazebo)

* Step_2. Create a start file in "/YCH_drone/drone/ekf2/iris_3"
    * Copy the existing iris rcs startup file, (iris_1 or iris_2), rename it to iris_3
    * Change "MAV_SYS_ID" to '3'
    * Change "SITL_UDP_PRT" is same as "mavlink_udp_port".
    * 1_st port of "mavlink start" is same as "mavlink stream"  (for QGC)
    * 2_nd port of "mavlink start" is same as 2_nd port of "fcu_url"
'''

# ************************* Design Agent Start ************************* #
class myAgent():
    def __init__(self):
        self.init_time = time.time()

    def select_action_set(self,env_set, vir_drone, vir_drone_pos):
        # Movement design
        command_X = 0.75
        command_Y = 0
        command_Z = 0
        command_Yaw = 0.15
        t_pos_x = math.cos(vir_drone_pos[1][2])*command_X - math.sin(vir_drone_pos[1][2])*command_Y
        t_pos_y = math.sin(vir_drone_pos[1][2])*command_X + math.cos(vir_drone_pos[1][2])*command_Y

        target_pos = vir_drone_pos + np.array([[t_pos_x,t_pos_y,command_Z],[0,0,command_Yaw]])

        # Calculate Error
        formation = vir_drone.form_set[vir_drone.drone_num-1] 
        body_pos_set = vir_drone.cal_body_frame(target_pos, env_set)
        error_set = []
        for i in range(len(body_pos_set)):
            bC_x = formation[i][0][0] - body_pos_set[i][0][0]
            bC_y = formation[i][0][1] - body_pos_set[i][0][1]
            # target point rotate (以無人機視角看目標點)
            C_x = math.cos(-body_pos_set[i][1][2])*bC_x - math.sin(-body_pos_set[i][1][2])*bC_y
            C_y = math.sin(-body_pos_set[i][1][2])*bC_x + math.cos(-body_pos_set[i][1][2])*bC_y
            C_z = formation[i][0][2] - body_pos_set[i][0][2] 
            C_yaw = normalize_angle(formation[i][1][2]-body_pos_set[i][1][2])
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
        1. 虛擬無人機為 golbal frame 此資料來自 所有實體無人機的型心
        2. 虛擬無人機主要是以位置與姿態控制，而非速度控制，這樣比較好保持隊形整齊
        3. 也就是輸入值為虛擬無人機期望到達的位置與姿態
        4. 控制主要在body frame當中控制
        5. 也就是回傳的值是僚機在虛擬無人機body frame的座標
        6. 僚機有bodyframe的座標後，再進行速度控制(PID)
        7. 編隊的型心一定的虛擬機的座標原點，也就是所有直相加等於零
        '''
        self.drone_num = drone_num
        form_1 = np.array([[[0,0,0],[0,0,0]]])
        form_2 = np.array([[[0,2,0],[0,0,0]],
                             [[0,-2,0],[0,0,0]]])
        form_3 = np.array([[[-2,0,0],[0,0,0]],
                             [[1,2,0],[0,0,0]],
                             [[1,-2,0],[0,0,0]]])
        form_4 = np.array([[[-2,0,0],[0,0,0]],
                            [[1,2,0],[0,0,0]],
                            [[1,-2,0],[0,0,0]],
                            [[0,0,0],[0,0,0]]])
        self.form_set = [form_1, form_2, form_3, form_4]
        self.error_sum_set = []
        self.error_past_set = []
        for i in range(drone_num):
            error_sum = np.array([[0.,0.,0.],[0.,0.,0.]])
            error_past = np.array([[0.,0.,0.],[0.,0.,0.]])
            self.error_sum_set.append(error_sum)
            self.error_past_set.append(error_past)

    def cal_pos(self, env_set):
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
            diff_yaw = env_set[i]._PoseStamped2np(env_set[i].current_local_pos)[1][2]-tar_global_pos[1][2]
            diff_yaw = normalize_angle(diff_yaw)

            body_pos_x = math.cos(-global_yaw)*diff_pos[0] - math.sin(-global_yaw)*diff_pos[1]
            body_pos_y = math.sin(-global_yaw)*diff_pos[0] + math.cos(-global_yaw)*diff_pos[1]
            body_pos = np.array([[body_pos_x, body_pos_y, diff_pos[2]],[0,0,diff_yaw]])
            body_pos_set.append(body_pos)

        return body_pos_set

    def vel_PID_control(self, error_set, dt):
        control_input_set = []
        kp_linear, ki_linear, kd_linear = 3.0, 0.5*(1.1**2), 0.1*(1.1**5)
        kp_rotate, ki_rotate, kd_rotate = 1.0, 0.0, 0.0
        linear_limit = 20  # 20 m/s = 72km/hr
        rotate_limit = math.pi/4  # rad/s
        for i in range(self.drone_num):
            # X-dir
            e_x = error_set[i][0][0]

            self.error_sum_set[i][0][0] = self.error_sum_set[i][0][0]*0.9 + e_x*dt
            e_x_sum = self.error_sum_set[i][0][0]
            e_x_past = self.error_past_set[i][0][0]

            input_X = kp_linear*e_x + ki_linear*e_x_sum + kd_linear*((e_x-e_x_past)/dt)
            input_X = max(min(input_X,linear_limit),-linear_limit)
            self.error_past_set[i][0][0] = e_x
            
            # Y-dir
            e_y = error_set[i][0][1]
            
            self.error_sum_set[i][0][1] = self.error_sum_set[i][0][1]*0.9 + e_y*dt
            e_y_sum = self.error_sum_set[i][0][1]
            e_y_past = self.error_past_set[i][0][1]
            
            input_Y = kp_linear*e_y + ki_linear*e_y_sum + kd_linear*((e_y-e_y_past)/dt)
            input_Y = max(min(input_Y,linear_limit),-linear_limit)
            self.error_past_set[i][0][1] = e_y
            
            # Z-dir
            e_z = error_set[i][0][2]
            
            self.error_sum_set[i][0][2] = self.error_sum_set[i][0][2]*0.9 + e_z*dt
            e_z_sum = self.error_sum_set[i][0][2]
            e_z_past = self.error_past_set[i][0][2]
            
            input_Z = kp_linear*e_z + ki_linear*e_z_sum + kd_linear*((e_z-e_z_past)/dt)
            input_Z = max(min(input_Z,linear_limit),-linear_limit)
            self.error_past_set[i][0][2] = e_z
            
            # Yaw-dir
            e_yaw = error_set[i][1][2]
            
            self.error_sum_set[i][1][2] = self.error_sum_set[i][1][2]*0.9 + e_yaw*dt
            e_yaw_sum = self.error_sum_set[i][1][2]
            e_yaw_past = self.error_past_set[i][1][2]
            
            input_Yaw = kp_rotate*e_yaw + ki_rotate*e_yaw_sum + kd_rotate*((e_yaw-e_yaw_past)/dt)
            input_Yaw = max(min(input_Yaw,rotate_limit),-rotate_limit)
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
        formation = vir_drone.form_set[vir_drone.drone_num-1] 
        body_pos_set = vir_drone.cal_body_frame(init_position, env_set)        
        condition_set = True
        error_set = []
        linear_limit = 2.0  # m
        for i in range(num_env):
            bC_x = formation[i][0][0] - body_pos_set[i][0][0]
            bC_y = formation[i][0][1] - body_pos_set[i][0][1]
            # target point rotate (以無人機視角看目標點)
            C_x = math.cos(-body_pos_set[i][1][2])*bC_x - math.sin(-body_pos_set[i][1][2])*bC_y
            C_y = math.sin(-body_pos_set[i][1][2])*bC_x + math.cos(-body_pos_set[i][1][2])*bC_y
            C_z = formation[i][0][2] - body_pos_set[i][0][2] 
            C_yaw = normalize_angle(formation[i][1][2]-body_pos_set[i][1][2])

            # error limit (防止震盪過大)
            C_distant = (C_x**2+C_y**2+C_z**2)**0.5
            if C_distant > linear_limit:
                discont_rate = C_distant/linear_limit
            else:
                discont_rate = 1
            C_x = C_x/discont_rate
            C_y = C_y/discont_rate
            C_z = C_z/discont_rate
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
        env_set[i].observation.local_pose = env_set[i]._PoseStamped2np(env_set[i].current_local_pos)
        env_set[i].observation.img = env_set[i].current_img
        observation_set.append(env_set[i].observation)
    

    vir_drone_pos = vir_drone.cal_pos(env_set)
    print("[State] : Reset Done")
    return vir_drone_pos

def velocity_step_set(env_set, vir_drone, action_set):     
        next_observation_set = []
        done_set = True
        for i in range(len(env_set)):
            next_observation, reward, done, info = env_set[i].velocity_step(action_set[i])
            next_observation_set.append(next_observation)
            done_set = done_set and done
        next_vir_drone_pos = vir_drone.cal_pos(env_set)
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
        init_position = np.array([[-6,-6,3.0],[0,0,0*math.pi/4]])
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
    parser.add_argument("--drone_num", default=4, type=int, help="Total Drone Number")
    args = parser.parse_args()
    main(args)