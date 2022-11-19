import numpy
import argparse
from env.main_enviroment import Drone_Enviroment as ENV

# ************************* Design Agent Start ************************* #
class myAgent():
    def __init__(self):
        pass

    def select_action(obs):
        action = [x_dir, y_dir, z_dir]
        return action

# ************************* Design Agent Stop  ************************* #

def main(args):
    env = ENV()
    agent = myAgent()
    
    for ep in range(10):
        obsevation = env.reset()
        while True:
            action = agent.select_action(obsevation)
            next_obsevation, reward, done, info = env.step(action)
            obsevation = next_obsevation
            if done:
                break
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default=1.0, type=float, help="Just For Test")
    args = parser.parse_args()
    main(args)