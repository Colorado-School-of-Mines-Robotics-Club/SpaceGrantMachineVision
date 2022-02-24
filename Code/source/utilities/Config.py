# Built in python libs
import sys
import os
from typing import Dict
import json

# Additional libs
import cv2
import numpy as np

# Custom imports
from source.logger.Logger import Logger


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
    wlsParams = dict()
    filepaths = dict()
    hardwarePorts = dict()

    @classmethod
    def init(cls, configFile="config.json"):
        try:
            with open(configFile, 'r') as f:
                cls.data = json.load(f)
            cls.runParameters = cls.data['run_parameters']
            cls.loggingOptions = cls.data['logging_options']
            cls.iterationConstants = cls.data['iteration_constants']
            cls.cameraPorts = cls.data['camera_ports']
            cls.orbParams = cls.data['orb_params']
            cls.featureParams = cls.data['feature_params']
            cls.objectDetectionParams = cls.data['object_detection_params']
            cls.sgbmParams = cls.data['sgbm_params']
            cls.wlsParams = cls.data['wls_filter_params']
            cls.filepaths = cls.data['file_paths']
            cls.hardwarePorts = cls.data['hardware_ports']
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
    def getCameraPorts(cls) -> (int, int):
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
    def getSBGMParamsDict(cls) -> Dict:
        return cls.sgbmParams

    @classmethod
    def getWLSParamsDict(cls) -> Dict:
        return cls.wlsParams

    @classmethod
    def getFilepathsDict(cls) -> Dict:
        return cls.filepaths

    @classmethod
    def getHardwarePortsDict(cls) -> Dict:
        return cls.hardwarePorts

