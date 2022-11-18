import threading
from pynput import keyboard

'''
REF : https://blog.csdn.net/Gu_fCSDN/article/details/104708248
'''

Done = False

def on_press(key):
    try:
        if key.char == 'w':
            print("W")
        elif key.char == 'a':
            print("A")
        elif key.char == 'd':
            print("D")
        elif key.char == 's':
            print("S")
    
    except AttributeError:
        pass

def on_release(key):
    # print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def keyboard_thread():
    global Done
    # Collect events until released
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    Done = True

def main():
    global Done
    keyboard_t = threading.Thread(target = keyboard_thread)
    keyboard_t.start()

    print("HAHAHA")

    # Done
    keyboard_t.join()


if __name__ == "__main__":
    main()