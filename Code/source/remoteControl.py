import cv2
import numpy as np
import time


def remoteControl(hz: float):
    dummy_image = np.zeros((640, 480))
    while True:
        # pattern match and/or use if, elif, else (no pattern matching since need support for python 3.6)
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            print('You pressed %d (0x%x), 2LSB: %d (%s)' % (key, key, key % 2 ** 16,
                                                        repr(chr(key % 256)) if key % 256 < 128 else '?'))

        if key == ord('w'):
            print(key)
        elif key == ord('a'):
            print(key)
        elif key == ord('s'):
            print(key)
        elif key == ord('d'):
            print(key)
        elif key == ord('x'):
            print(key)
        elif key == 27:
            print("Quitting program")
            break
        else:
            pass

        cv2.imshow("Input Screen", dummy_image)
        # time.sleep(1.0 / hz)
        cv2.waitKey(1)
