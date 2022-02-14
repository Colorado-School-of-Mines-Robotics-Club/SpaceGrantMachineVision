from HardwareManager import HardwareManager
import multiprocessing as mp
def startHardwareManager():
    #spawn hardware class as a seperate process
    hardware = mp.Process(target=HardwareManager, args=(0,))
    hardware.start()


