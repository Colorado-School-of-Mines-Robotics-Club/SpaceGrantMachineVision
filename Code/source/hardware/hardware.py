from HardwareManager import HardwareManager
import multiprocessing as mp


def startHardwareManager():
    # create HardwareManager
    hardware = HardwareManager()
    hardware.start_threads()


def getXBee():
    return 10.0, 15.0
