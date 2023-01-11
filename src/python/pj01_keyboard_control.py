import threading
import tkinter
import copy
import time
import math
import cv2
import numpy as np
from PIL import Image, ImageTk
from env.main_enviroment import Drone_Enviroment as ENV

'''
Tutorial:

w : forward
s : go back
a : left
d : right

i : up
k : down
j : counterClockwise
l : Clockwise

p : lock the drone
n : unlock the drone
h : back to inital point

'''

Done = False
observation = None
action = None
press_time = time.time()
drone_lock = True
already_locked = False
back_to_home = False

class TK_KeyBoardThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.start_control = False
    
    def xFunc_press(self, event):
        # forward / go_back / left / right
        global action, press_time, drone_lock, already_locked, back_to_home
        if action is not None and self.start_control:
            
            # forward / back / left / right
            if event.char == 'w':
                if action[0][0] < 5: 
                    action[0][0]+=0.1
            elif event.char == 's':
                if action[0][0] > -5: 
                    action[0][0]-=0.1
            elif event.char == 'a':
                if action[0][1] < 5: 
                    action[0][1]+=0.1
            elif event.char == 'd':
                if action[0][1] > -5: 
                    action[0][1]-=0.1
            
            # up / down / counterClockwise /Clockwise
            elif event.char == 'i':
                if action[0][2] < 5: 
                    action[0][2]+=0.1
            elif event.char == 'k':
                if action[0][2] > -5: 
                    action[0][2]-=0.1
            elif event.char == 'j':
                if action[1][2] < 0.5: 
                    action[1][2]+=0.1
            elif event.char == 'l':
                if action[1][2] > -0.5: 
                    action[1][2]-=0.1
            
            # drone lock
            elif event.char == 'p':
                drone_lock = True 
            
            elif event.char == 'n':
                drone_lock = False
                already_locked = False

            elif event.char == 'h':
                back_to_home = True
                drone_lock = True 
                already_locked = False
            
            press_time = time.time()

    def run(self):
        global Done
        # Collect events until released
        win = tkinter.Tk()
        win.title("KeyBoard Controller")
        win.geometry("640x360")

        img = Image.open('img/controller_640x360.jpg')
        tk_img = ImageTk.PhotoImage(img)
        label = tkinter.Label(win, image=tk_img, width=640, height=360)
        label.pack()
        win.bind("<KeyPress>", self.xFunc_press)
        win.mainloop()
        Done = True


def main():
    global Done
    global action, press_time, drone_lock, already_locked, back_to_home

    env = ENV("/uav1")
    observation = env.reset()

    KB_T = TK_KeyBoardThread()
    KB_T.start()

    action = np.array([[0,0,0],[0,0,0]])
    KB_T.start_control = True

    # Control Loop
    while True:
        # 1-1. Drone is locked
        if drone_lock:
            if already_locked:
                pass 
            else :
                old_observation = copy.copy(observation)
                already_locked = True

            print("Drone is locked, press [n-key] to unlock !! , press [p-key] to lock again !!   ", end='\r')
            next_observation, reward, done, info = env.position_step(old_observation.local_pose)
            observation = next_observation

        # 1-2. Drone is unlocked (Start Control)
        else:
            # fly
            if (time.time()-press_time)>0.1 :
                action = action/4
                action[1][2] = 0

            elif (time.time()-press_time)>0.5 :
                action = np.array([[0,0,0],[0,0,0]])

            print("Action XYZyaw : {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(action[0][0], action[0][1], action[0][2], action[1][2]), end='\r')
            next_observation, reward, done, info = env.velocity_step(action)
            observation = next_observation
        
        # 2. Drone back to home
        if back_to_home:
            observation = env.reset()
            drone_lock = True
            already_locked = False
            back_to_home = False
        
        # Show IMG
        '''
        cur_img = observation.img
        cv2.imshow("Image window", cur_img)
        cv2.waitKey(3)
        '''

        if Done:
            break

    # wait for KeyBoardThread done 
    cv2.destroyAllWindows()
    KB_T.join()
    env.reset()
    env.shotdown()


if __name__ == "__main__":
    main()