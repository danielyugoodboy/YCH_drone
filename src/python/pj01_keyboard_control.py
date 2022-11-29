import threading
import tkinter
import copy
import time
import math
import cv2
import numpy as np
from PIL import Image, ImageTk
from env.main_enviroment import Drone_Enviroment as ENV

Done = False
observation = None
action = None
press_time = time.time()

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
        global action, press_time
        if action is not None and self.start_control:

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
            
            press_time = time.time()

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
    global action, press_time

    env = ENV()
    observation = env.reset()

    KB_T = TK_KeyBoardThread()
    KB_T.start()

    action = np.array([[0,0,0],[0,0,0]])
    KB_T.start_control = True

    # Control Loop
    while True:
        # make drone stable
        if (time.time()-press_time)>0.1 :
            action = action/4
            action[1][2] = 0

        elif (time.time()-press_time)>0.5 :
            action = np.array([[0,0,0],[0,0,0]])
        
        cur_img = observation.img
        cv2.imshow("Image window", cur_img)
        cv2.waitKey(3)

        #print("Action XYZyaw : {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(action[0][0], action[0][1], action[0][2], action[1][2]), end='\r')
        next_observation, reward, done, info = env.velocity_step(action)
        observation = next_observation
        if Done:
            break

    # wait for KeyBoardThread done 
    cv2.destroyAllWindows()
    KB_T.join()
    env.reset()
    env.shotdown()


if __name__ == "__main__":
    main()