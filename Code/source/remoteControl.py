from threading import Thread
import time
import sys
from dataclasses import dataclass
import keyboard


@dataclass
class RobotData:
    linear: float
    angular: float
    fl_height: float
    fr_height: float
    bl_height: float
    br_height: float

    def incrementAllHeights(self, delta):
        self.fl_height += delta
        self.fr_height += delta
        self.bl_height += delta
        self.br_height += delta

    def roundData(self):
        self.linear = round(self.linear, 1)
        self.angular = round(self.angular, 1)
        self.fl_height = round(self.fl_height, 1)
        self.fr_height = round(self.fr_height, 1)
        self.bl_height = round(self.bl_height, 1)
        self.br_height = round(self.br_height, 1)


vel_data = RobotData(linear=0.0, angular=0.0, fl_height=0.0, fr_height=0.0, bl_height=0.0, br_height=0.0)
DELTA_VEL = 0.1
DELTA_ANG = 5.0
DELTA_HEIGHT = 1.0
key_state = {}
global_shutdown = False


def key_update(key, state):
    # key = key.name
    # # key is pressed for the first time
    # if key not in key_state:
    #     key_state[key] = state
    #     return True
    #
    # # key changed state
    # if state != key_state[key]:
    #     key_state[key] = state
    #     return True
    #
    # # no change
    # return False
    return True


def key_press(key):
    if key.name == 'esc':
        global global_shutdown
        global_shutdown = True
        print('\nPress Ctrl+C to exit')
        return False

    k = key.name
    # print(f"Pressed {k}")

    # check if press changes state
    change = key_update(key, True)
    if change:
        global vel_data, DELTA_VEL, DELTA_ANG, DELTA_HEIGHT
        if k in ['w', 'up']:
            vel_data.linear += DELTA_VEL
        elif k in ['s', 'down']:
            vel_data.linear -= DELTA_VEL
        elif k in ['d', 'right']:
            vel_data.angular -= DELTA_ANG
        elif k in ['a', 'left']:
            vel_data.angular += DELTA_ANG
        elif k in ['x']:
            DELTA_VEL += 0.1
        elif k in ['z']:
            DELTA_VEL -= 0.1
        elif k in ['t']:
            vel_data.incrementAllHeights(DELTA_HEIGHT)
        elif k in ['g']:
            vel_data.incrementAllHeights(-DELTA_HEIGHT)
        elif k in ['y']:
            vel_data.fl_height += DELTA_HEIGHT
        elif k in ['h']:
            vel_data.fl_height -= DELTA_HEIGHT
        elif k in ['u']:
            vel_data.fr_height += DELTA_HEIGHT
        elif k in ['j']:
            vel_data.fr_height -= DELTA_HEIGHT
        elif k in ['i']:
            vel_data.bl_height += DELTA_HEIGHT
        elif k in ['k']:
            vel_data.bl_height -= DELTA_HEIGHT
        elif k in ['o']:
            vel_data.br_height += DELTA_HEIGHT
        elif k in ['l']:
            vel_data.br_height -= DELTA_HEIGHT

    return True


def key_release(key):
    k = key.name
    # print(f"Released {k}")

    change = key_update(key, False)
    if change:
        global vel_data
        if k in ['w', 'up']:
            vel_data.linear = 0
        elif k in ['s', 'down']:
            vel_data.linear = 0
        elif k in ['d', 'right']:
            vel_data.angular = 0
        elif k in ['a', 'left']:
            vel_data.angular = 0
        elif k in ['x']:
            pass
        elif k in ['z']:
            pass
        elif k in ['t']:
            pass
        elif k in ['g']:
            pass
        elif k in ['y']:
            pass
        elif k in ['h']:
            pass
        elif k in ['u']:
            pass
        elif k in ['j']:
            pass
        elif k in ['i']:
            pass
        elif k in ['k']:
            pass
        elif k in ['o']:
            pass
        elif k in ['l']:
            pass
    return True


def user_display(hz: float = 60.0) -> None:
    instructions = 'Use WASD or ARROWS to control.\n'
    instructions += 'Use x/z to increase/decrease speed.\n'
    instructions += 'Use c/v to increase/decrease turning radius.\n'
    instructions += 'Use t/g to raise and lower all ride heights.\n'
    instructions += 'Wheel ride heights can be adjusted in pairs with y/h, u/j, i/k, o/l.\n'
    print(instructions)
    while True:
        try:
            print('\r' + ' ' * 80, end='')
            sys.stdout.flush()
            global vel_data
            log_str = f"\r\t\tLINEAR: {vel_data.linear}\tANGULAR: {vel_data.angular}\t" \
                      f"HEIGHTS -> FL: {vel_data.fl_height}, FR: {vel_data.fr_height}, BL: {vel_data.bl_height}, " \
                      f"BR: {vel_data.br_height}\t\tCURRENT SPEED: {DELTA_VEL}"
            print(log_str, end=' ')
            sys.stdout.flush()

            global global_shutdown
            if global_shutdown:
                exit(0)
            else:
                time.sleep(1.0 / hz)
                # publish key message to handler for robotic model
                pass

        except KeyboardInterrupt:
            exit(0)


def remoteControl(hz: float = 60.0) -> None:
    # start key listener thread
    keyboard.on_press(key_press)
    keyboard.on_release(key_release)

    # start user display thread
    display_thread = Thread(target=user_display)
    display_thread.start()

    # spin the function to run the threads
    while not global_shutdown:
        vel_data.roundData()
        global DELTA_VEL
        DELTA_VEL = round(DELTA_VEL, 1)
        time.sleep(1.0 / hz)

    # join the threads
    display_thread.join()
