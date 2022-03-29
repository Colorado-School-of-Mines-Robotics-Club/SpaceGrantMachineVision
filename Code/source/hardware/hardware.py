from HardwareManager import HardwareManager
import multiprocessing as mp


def startHardwareManager():
    # spawn hardware class as a seperate process
    hardware = mp.Process(target=HardwareManager)
    hardware.start()


def getXBee():
    return 10.0, 15.0
