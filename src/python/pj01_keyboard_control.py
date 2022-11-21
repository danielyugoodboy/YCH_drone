import threading
import tkinter
import argparse
from PIL import Image, ImageTk

Done = False

class TK_KeyBoardThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
    
    def xFunc(self, event):
        # forward / go_back / left / right
        if event.char == 'w':
            print("W")
        elif event.char == 's':
            print("S")
        elif event.char == 'a':
            print("A")
        elif event.char == 'd':
            print("D")
        
        # up / down / counterClockwise /Clockwise
        elif event.char == 'i':
            print("I")
        elif event.char == 'k':
            print("K")
        elif event.char == 'j':
            print("J")
        elif event.char == 'l':
            print("L")

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

        win.bind("<Key>", self.xFunc)
        win.mainloop()
        
        Done = True


def main(args):
    global Done
    KB_T = TK_KeyBoardThread()
    KB_T.start()
    print("Done situation: {}".format(Done))

    # wait for KeyBoardThread done 
    KB_T.join()
    print("Done situation: {}".format(Done))
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default=1.0, type=float, help="Just For Test")
    args = parser.parse_args()
    main(args)