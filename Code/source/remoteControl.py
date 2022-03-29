import cv2
import numpy as np
import time
from threading import Thread
import time
import sys
from typing import Tuple
from pynput import keyboard


DELTA_VEL = 0.1
DELTA_ANG = 5.0
key_state = {}
global_shutdown = False


def key_update(key, state):
    # key is pressed for the first time
    if key not in key_state:
        key_state[key] = state
        return True

    # key changed state
    if state != key_state[key]:
        key_state[key] = state
        return True

    # no change
    return False


def key_press(key):
    if key == keyboard.Key.esc:
        global global_shutdown
        global_shutdown = True
        print('\nPress Ctrl+C to exit')
        return False
    try:
        k = key.char
    except AttributeError:
        k = key.name

    # check if press changes state
    change = key_update(key, True)
    if change:
        global vel_msg, DELTA_VEL, DELTA_ANG
        if k in ['w', 'up']:
            vel_msg.y += DELTA_VEL
        elif k in ['s', 'down']:
            vel_msg.y -= DELTA_VEL
        elif k in ['d', 'right']:
            vel_msg.x += DELTA_VEL
        elif k in ['a', 'left']:
            vel_msg.x -= DELTA_VEL
        elif k in ['e']:
            vel_msg.theta -= DELTA_ANG
        elif k in ['q']:
            vel_msg.theta += DELTA_ANG
        elif k in ['x']:
            DELTA_VEL += 0.1
        elif k in ['z']:
            DELTA_VEL -= 0.1
    return True


def key_release(key):
    try:
        # character input
        k = key.char
    except:
        # arrow key/other input
        k = key.name

    change = key_update(key, False)
    if change:
        global vel_msg
        if k in ['w', 'up']:
            vel_msg.y = 0
        elif k in ['s', 'down']:
            vel_msg.y = 0
        elif k in ['d', 'right']:
            vel_msg.x = 0
        elif k in ['a', 'left']:
            vel_msg.x = 0
        elif k in ['e']:
            vel_msg.theta = 0
        elif k in ['q']:
            vel_msg.theta = 0
        elif k in ['x']:
            pass
        elif k in ['z']:
            pass
    return True


def user_display(vel_msg: Tuple[float, float], global_shutdown: bool, hz: float = 60.0) -> None:
    print(
        'Use WASD or the ARROW KEYS to control.\nUse x/z to increase/decrease speed')
    while True:
        try:
            print('\r' + ' ' * 80, end='')
            sys.stdout.flush()
            log_str = "\r\t\tX: {}\tY: {}\tTHETA: {}\t".format(vel_msg.x,
                                                               vel_msg.y,
                                                               vel_msg.theta)
            print(log_str, end=' ')
            sys.stdout.flush()

            global stop_display
            if stop_display:
                exit(0)

            if not global_shutdown:
                time.sleep(1.0 / hz)
                # publish key message to handler for robotic model
            else:
                exit(0)
        except KeyboardInterrupt:
            exit(0)


def remoteControl(hz: float = 60.0) -> None:
    # start key listener thread
    key_listener = keyboard.Listener(on_press=key_press, on_release=key_release)
    key_listener.start()

    # start user display thread
    display_thread = Thread(target=user_display, args=(global_shutdown,))
    display_thread.start()

    # spin the function to run the threads
    while not global_shutdown:
        time.sleep(1.0 / hz)

    # join the threads
    key_listener.join()
    display_thread.join()
