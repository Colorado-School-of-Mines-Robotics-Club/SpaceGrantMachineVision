import cv2
import numpy as np
import time
from threading import Thread
import time
import sys
from dataclasses import dataclass
from typing import Tuple
from pynput import keyboard


@dataclass
class VelocityData:
    linear: float
    angular: float


vel_data = VelocityData(linear=0.0, angular=0.0)
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
        global vel_data, DELTA_VEL, DELTA_ANG
        if k in ['w', 'up']:
            vel_data.linear += DELTA_VEL
        elif k in ['s', 'down']:
            vel_data.linear -= DELTA_VEL
        elif k in ['d', 'right']:
            vel_data.linear += DELTA_VEL
        elif k in ['a', 'left']:
            vel_data.linear -= DELTA_VEL
        elif k in ['e']:
            vel_data.angular -= DELTA_ANG
        elif k in ['q']:
            vel_data.angular += DELTA_ANG
        elif k in ['x']:
            DELTA_VEL += 0.1
        elif k in ['z']:
            DELTA_VEL -= 0.1
    return True


def key_release(key):
    try:
        # character input
        k = key.char
    except AttributeError:
        # arrow key/other input
        k = key.name

    change = key_update(key, False)
    if change:
        global vel_data
        if k in ['w', 'up']:
            vel_data.linear = 0
        elif k in ['s', 'down']:
            vel_data.linear = 0
        elif k in ['d', 'right']:
            vel_data.linear = 0
        elif k in ['a', 'left']:
            vel_data.linear = 0
        elif k in ['e']:
            vel_data.angular = 0
        elif k in ['q']:
            vel_data.angular = 0
        elif k in ['x']:
            pass
        elif k in ['z']:
            pass
    return True


def user_display(hz: float = 60.0) -> None:
    print(
        'Use WASD or the ARROW KEYS to control.\nUse x/z to increase/decrease speed')
    while True:
        try:
            print('\r' + ' ' * 80, end='')
            sys.stdout.flush()
            global vel_data
            log_str = "\r\t\tLINEAR: {}\tANGULAR: {}\t".format(vel_data.linear,
                                                               vel_data.angular)
            print(log_str, end=' ')
            sys.stdout.flush()

            global global_shutdown
            if global_shutdown:
                exit(0)
            else:
                time.sleep(1.0 / hz)
                # publish key message to handler for robotic model

        except KeyboardInterrupt:
            exit(0)


def remoteControl(hz: float = 60.0) -> None:
    # start key listener thread
    key_listener = keyboard.Listener(on_press=key_press, on_release=key_release)
    key_listener.start()

    # start user display thread
    display_thread = Thread(target=user_display)
    display_thread.start()

    # spin the function to run the threads
    while not global_shutdown:
        time.sleep(1.0 / hz)

    # join the threads
    key_listener.join()
    display_thread.join()
