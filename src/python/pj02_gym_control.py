import numpy as np
import argparse
from env.main_enviroment import Drone_Enviroment as ENV

# ************************* Design Agent Start ************************* #
class myAgent():
    def __init__(self):
        pass

    def select_action(self,obs):
        x_dir = 2
        y_dir = 0
        z_dir = 0
        pitch = 0
        row = 0
        yaw = 0

        action = np.array([[x_dir, y_dir, z_dir], [pitch, row, yaw]])
        return action

# ************************* Design Agent Stop  ************************* #
'''
0. action :
numpy array
shape = (2, 3)
np.array([[x,y,z],[pitch, roll, yaw]])

1. observation.local_pose :
numpy array
shape = (2, 3)
np.array([[x,y,z],[pitch, roll, yaw]])

2. observation.global_pose :
numpy array
shape = (3,)
np.array([x,y,z])

3. observation.img :
numpy array
shape = (240, 320, 3)
'''
def main(args):
    env = ENV("/uav1")
    agent = myAgent()
    
    for ep in range(2):
        observation = env.reset()
        print("Start Episode : {}".format(ep+1))
        while True:
            action = agent.select_action(observation)
            next_observation, reward, done, info = env.velocity_step(action)
            observation = next_observation
            if done:
                break
    
    env.reset()
    env.shotdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default=1.0, type=float, help="Just For Test")
    args = parser.parse_args()
    main(args)