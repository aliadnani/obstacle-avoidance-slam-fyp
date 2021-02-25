from pynput import keyboard
import os

currently_pressed_keys = set()

def print_robot_instruction():
    if len(currently_pressed_keys) == 0:
        print('[0,0,0]')
    else:
        print("you pressed something!!")
    return

def on_press(key):
    try:
        if str(key.char) not in currently_pressed_keys:
            currently_pressed_keys.add(str(key.char))
            print_robot_instruction()
            # print(currently_pressed_keys)
    except AttributeError:
        print('special key {0} pressed'.format(key))

def on_release(key):
    if str(key.char) in currently_pressed_keys:
        currently_pressed_keys.remove(str(key.char))
        # os.system('cls' if os.name == 'nt' else 'clear')
        # print(currently_pressed_keys)
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    print_robot_instruction()

# a
with keyboard.Listener( on_press=on_press, on_release=on_release, supress=True) as listener:
    listener.join()