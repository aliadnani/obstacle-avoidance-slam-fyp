from pynput.keyboard import Key, Listener

c = []
w = [1,0,0]
s = [-1,0,0]
a = [0,0,1]
d = [0,0,-1]

def on_press(key):
    if str(key.char) == 'w':
        c.append(w)
        print(w)
    elif str(key.char) == 's':
        c.append(s)
        print(s)
    elif str(key.char) == 'a':
        c.append(a)
        print(a)
    elif str(key.char) == 'd':
        c.append(d)
        print(d)
    elif str(key.char) == 'p':
        print(*c, sep='\n')
    else:
        pass



def on_release(key):
    pass
    if key == Key.esc:
        # Stop listener
        return False

# Collect events until released
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()