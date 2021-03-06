# Built in python libs
import sys
import os
from typing import Dict, Tuple
import json

# Additional libs

# Custom imports


class Config:
    data = dict()
    runParameters = dict()
    loggingOptions = dict()
    iterationConstants = dict()
    cameraPorts = dict()
    orbParams = dict()
    featureParams = dict()
    objectDetectionParams = dict()
    sgbmParams = dict()
    odometerParams = dict()
    wlsParams = dict()
    filepaths = dict()
    electronicPorts = dict()
    dimensions = dict()
    trim = dict()

    @classmethod
    def init(cls):
        try:
            try:
                with open("config.json", 'r') as f:
                    cls.data = json.load(f)
            except FileNotFoundError:
                os.chdir('..\\Code')
                with open("config.json", 'r') as f:
                    cls.data = json.load(f)
            cls.runParameters = cls.data['run_parameters']
            cls.loggingOptions = cls.data['logging_options']
            cls.iterationConstants = cls.data['iteration_constants']
            cls.cameraPorts = cls.data['camera_ports']
            cls.orbParams = cls.data['orb_params']
            cls.featureParams = cls.data['feature_params']
            cls.objectDetectionParams = cls.data['object_detection_params']
            cls.sgbmParams = cls.data['sgbm_params']
            cls.odometerParams = cls.data['odometer_params']
            cls.wlsParams = cls.data['wls_filter_params']
            cls.filepaths = cls.data['file_paths']
            cls.trim = cls.data['servo_trim']
            cls.electronicPorts = cls.data['electronics']
            cls.dimensions = cls.data['dimensions']
        except FileNotFoundError:
            raise FileNotFoundError("Cannot read config file")

    @classmethod
    def getConfig(cls) -> Dict:
        return cls.data

    @classmethod
    def getRunParameters(cls) -> Dict:
        return cls.runParameters

    @classmethod
    def getLoggingOptions(cls) -> Dict:
        return cls.loggingOptions

    @classmethod
    def getIterationConstantsDict(cls) -> Dict:
        return cls.iterationConstants

    @classmethod
    def getCamerasDict(cls) -> Dict:
        return cls.cameraPorts

    @classmethod
    def getCameraPorts(cls) -> Tuple[int, int]:
        return cls.cameraPorts['leftPort'], cls.cameraPorts['rightPort']

    @classmethod
    def getOrbParamsDict(cls) -> Dict:
        return cls.orbParams

    @classmethod
    def getFeatureParamsDict(cls) -> Dict:
        return cls.featureParams

    @classmethod
    def getObjectDetectionDict(cls) -> Dict:
        return cls.objectDetectionParams

    @classmethod
    def getSGBMParamsDict(cls) -> Dict:
        return cls.sgbmParams

    @classmethod
    def getOdometerParamsDict(cls) -> Dict:
        return cls.odometerParams

    @classmethod
    def getWLSParamsDict(cls) -> Dict:
        return cls.wlsParams

    @classmethod
    def getFilepathsDict(cls) -> Dict:
        return cls.filepaths

    @classmethod
    def getServoTrimDict(cls) -> Dict:
        return cls.trim

    @classmethod
    def getElectronicPortsDict(cls) -> Dict:
        # cls.electronicPorts['sensors']['accelerometer']['address'] =\
        #     hex(cls.electronicPorts['sensors']['accelerometer']['address'])
        # cls.electronicPorts['sensors']['accelerometer']['register'] =\
        #     hex(cls.electronicPorts['sensors']['accelerometer']['register'])
        return cls.electronicPorts

    @classmethod
    def getDimensionsDict(cls) -> Dict:
        return cls.dimensions

    @classmethod
    def getFrameSize(cls) -> Tuple[int, int]:
        frameSize = cls.cameraPorts['frame_size']
        return frameSize[0], frameSize[1]

    @classmethod
    def getMaxVel(cls) -> float:
        return cls.electronicPorts['utility']['max_vel']
