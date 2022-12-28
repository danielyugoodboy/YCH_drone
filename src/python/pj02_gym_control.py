import numpy as np
import argparse
from env.main_enviroment import Drone_Enviroment as ENV

# Design Agent Start
class myAgent():
    def __init__(self):
        pass

    def select_action(self,obs):
        x_dir, y_dir, z_dir = 2, 0, 0
        pitch, row, yaw = 0, 0, 0

        action = np.array([[x_dir, y_dir, z_dir], [pitch, row, yaw]])
        return action

# Gym like
def main(args):
    env = ENV("/uav1")
    agent = myAgent()
    
    for ep in range(args.ep):
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
    parser.add_argument("--ep", default=2, type=int, help="Episode Number")
    args = parser.parse_args()
    main(args)