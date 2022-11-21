import numpy as np
import argparse
from env.main_enviroment import Drone_Enviroment as ENV

# ************************* Design Agent Start ************************* #
class myAgent():
    def __init__(self):
        pass

    def select_action(self,obs):
        x_dir = obs[0][0]+1
        y_dir = obs[0][1]
        z_dir = obs[0][2]
        pitch = 0
        row = 0
        yaw = 0

        action = np.array([[x_dir, y_dir, z_dir], [pitch, row, yaw]])
        return action

# ************************* Design Agent Stop  ************************* #

def main(args):
    env = ENV()
    agent = myAgent()
    
    for ep in range(2):
        obsevation = env.reset()
        print("Start Episode : {}".format(ep+1))
        while True:
            action = agent.select_action(obsevation)
            next_obsevation, reward, done, info = env.step(action)
            obsevation = next_obsevation
            if done:
                break
    
    env.reset()
    env.shotdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default=1.0, type=float, help="Just For Test")
    args = parser.parse_args()
    main(args)