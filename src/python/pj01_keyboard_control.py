import threading
import tkinter
import copy
import time
import math
import numpy as np
from PIL import Image, ImageTk
from env.main_enviroment import Drone_Enviroment as ENV

Done = False
obsevation = None
action = None
press_time = time.time()
command = False

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
        global obsevation, action, press_time, command
        if obsevation is not None and action is not None and self.start_control:
            yaw = obsevation[1][2]
            l_yaw = obsevation[1][2]+math.pi/2
            r_yaw = obsevation[1][2]-math.pi/2

            if event.char == 'w':
                action[0][0] = obsevation[0][0]+1*math.cos(yaw)
                action[0][1] = obsevation[0][1]+1*math.sin(yaw)
            elif event.char == 's':
                action[0][0] = obsevation[0][0]-1*math.cos(yaw)
                action[0][1] = obsevation[0][1]-1*math.sin(yaw)
            elif event.char == 'a':
                action[0][0] = obsevation[0][0]-1*math.cos(r_yaw)
                action[0][1] = obsevation[0][1]-1*math.sin(r_yaw)
            elif event.char == 'd':
                action[0][0] = obsevation[0][0]-1*math.cos(l_yaw)
                action[0][1] = obsevation[0][1]-1*math.sin(l_yaw)
            
            # up / down / counterClockwise /Clockwise
            elif event.char == 'i':
                action[0][2] = obsevation[0][2]+0.5
            elif event.char == 'k':
                action[0][2] = obsevation[0][2]-0.5
            elif event.char == 'j':
                action[1][2] = obsevation[1][2]+0.5
            elif event.char == 'l':
                action[1][2] = obsevation[1][2]-0.5
            
            press_time = time.time()
            command = True

    def run(self):
        global Done
        # Collect events until released
        win = tkinter.Tk()
        win.title("KeyBoard Controller")
        win.geometry("640x360")

        img = Image.open('img/controler_640x360.jpg')
        tk_img = ImageTk.PhotoImage(img)
        label = tkinter.Label(win, image=tk_img, width=640, height=360)
        label.pack()
        win.bind("<KeyPress>", self.xFunc_press)
        win.mainloop()
        Done = True


def main():
    global Done
    global obsevation, action, press_time, command

    KB_T = TK_KeyBoardThread()
    KB_T.start()

    env = ENV()
    obsevation = env.reset()
    action = obsevation.copy()
    KB_T.start_control = True

    # Control Loop
    while True:
        # make drone stable
        if (time.time()-press_time)>0.1 and command:
            action = obsevation.copy()
            command = False
        
        print("Action XYZyaw : {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(action[0][0], action[0][1], action[0][2], action[1][2]), end='\r')
        next_obsevation, reward, done, info = env.step(action)
        obsevation = next_obsevation
        if Done:
            break

    # wait for KeyBoardThread done 
    KB_T.join()
    env.reset()
    env.shotdown()

if __name__ == "__main__":
    main()