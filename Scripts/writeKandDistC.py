import numpy as np
import os


# Function to write K matrix and dist coeffs to npz files
# K matrix is a 3x3 and dist coeffs is of length 4
def writeKandDistNPZ(lk: np.ndarray, rk: np.ndarray, ld: np.ndarray, rd: np.ndarray):
    # Gets the path to the Calibration folder in data for any computer
    calibrationPath = "Data/Calibration/"
    while not os.path.isdir(calibrationPath):
        calibrationPath = "../" + calibrationPath
    # saves the np.arrays inputed to their respective files
    np.save(calibrationPath + "leftK.npy", lk)
    np.save(calibrationPath + "rightK.npy", rk)
    np.save(calibrationPath + "leftDistC.npy", ld)
    np.save(calibrationPath + "rightDistC.npy", rd)


if __name__ == '__main__' :
    lk = np.asarray([[720.37726966, 0, 353.54704879], [0, 723.92353677, 224.94785218], [0, 0, 1]])
    ld = np.asarray([[-0.23686867], [-0.46513602], [0.00350406], [0.00689797], [0.71701436]])
    rk = np.asarray([[750.41319062, 0, 360.56503544], [0, 752.95957805, 234.54811838], [0, 0, 1]])
    rd = np.asarray([[-0.27671195], [-0.53573313], [0.00872607], [0.00643696], [1.14287171]])
    writeKandDistNPZ(lk, rk, ld, rd)
