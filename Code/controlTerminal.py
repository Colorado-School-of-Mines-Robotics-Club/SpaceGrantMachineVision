from pydoc import cli
from threading import Thread
import time
import pickle
import socket
import struct
from threading import Thread
import sys
from dataclasses import dataclass
import keyboard
import os
import platform

from source.hardware.RobotData import RobotData

import cv2


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
        # print('\nPress Ctrl+C to exit')
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
                pass

        except KeyboardInterrupt:
            exit(0)


server_address = None
client_socket = None


def init_robot_connection(ip_address: str = "localhost", tcp_port: int = 9500):
    global server_address
    server_address = (ip_address, tcp_port)
    global client_socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    while True:
        try:
            client_socket.connect(server_address)
            print("Connected to server\n")
            break
        except ConnectionRefusedError:
            print("Could not connect to server. Retrying...")
            time.sleep(1.0)
            pass
        except TimeoutError:
            print("Could not connect to server. Connection timeout. Retrying...")
            time.sleep(1.0)
            pass


def send_robot_data():
    global client_socket
    global vel_data
    packet = pickle.dumps(vel_data.to_list())
    try:
        client_socket.sendall(packet)
    except ConnectionResetError:
        global global_shutdown
        global_shutdown = True
        print("Connection reset. Exiting...")
        pass
    return


def remoteViewer(ip_address: str = "localhost", tcp_port: int = 9501) -> None:
    print("Starting remote viewer client")

    global server_address
    server_address = (ip_address, tcp_port)

    global client_socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    window_open = False
    running = True
    while running and not global_shutdown:
        try:
            client_socket.connect(server_address)
            data = b""
            payload_size = struct.calcsize("Q")

            while running and not global_shutdown:
                while len(data) < payload_size:
                    packet = client_socket.recv(4096)
                    if not packet:
                        break
                    data += packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack("Q", packed_msg_size)[0]

                while len(data) < msg_size:
                    data += client_socket.recv(4096)
                frame_data = data[:msg_size]
                data = data[msg_size:]
                frame = pickle.loads(frame_data)
                cv2.imshow("Received video stream", frame)
                window_open = True
                key = cv2.waitKey(1) & 0xFF
                if key == 27: # Escape
                    running = False
                    break
            client_socket.close()
            
        except ConnectionRefusedError as e:
            # print("Connection refused, retrying...")
            pass
        except Exception as e:
            # print("Remote Viewer encountered an error, retrying...")
            pass

        if window_open:
            cv2.destroyWindow("Received video stream")
            print("Close window")
            window_open = False
        time.sleep(1.0)


def handle_sigint_signal(cls, sig, frame):
    print("Received SIGINT signal")
    global global_shutdown
    global_shutdown = True
    return


def handle_sigterm_signal(cls, sig, frame):
    print("Received SIGTERM signal")
    global global_shutdown
    global_shutdown = True
    return


def run_controller():
    hz = 60.0

    ip_address = sys.argv[1][4:] # Argument format is "-IP ip_address"
    print(f"Using ip address {ip_address}")

    viewer_thread = Thread(target=remoteViewer, args=(ip_address,), name="Remote_Viewer", daemon=True)
    # viewer_thread.start()

    init_robot_connection(ip_address=ip_address)

    # start key listener thread
    keyboard.on_press(key_press)
    keyboard.on_release(key_release)

    # start user display thread
    display_thread = Thread(target=user_display, daemon=True)
    display_thread.start()

    # spin the function to run the threads
    while not global_shutdown:
        vel_data.roundData()
        global DELTA_VEL
        DELTA_VEL = round(DELTA_VEL, 1)

        send_robot_data()

        time.sleep(1.0 / hz)

    global client_socket
    if client_socket is not None:
        client_socket.close()

    # join the threads
    display_thread.join()

    if viewer_thread.is_alive():
        print("Waiting on RemoteViewer")
        viewer_thread.join()


if __name__ == "__main__":

    if platform.system() == "Linux" and not os.getuid() == 0:
        print("This script must be run as root")
        sys.exit(-1)

    run_controller()
