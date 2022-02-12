# Built in python libs
import sys
import os
from typing import Dict
import json

# Additional libs
import cv2
import numpy as np

# Custom imports
from Code.source.logger.Logger import Logger


class Config:
    data = dict()
    iterationConstants = dict()
    cameraPorts = dict()
    orbParams = dict()
    sgbmParams = dict()
    filepaths = dict()
    hardwarePorts = dict()

    @classmethod
    def init(cls, configFile="config.json"):
        try:
            with open(configFile, 'r') as f:
                cls.data = json.load(f)
            cls.iterationConstants = cls.data['iteration_constants']
            cls.cameraPorts = cls.data['camera_ports']
            cls.orbParams = cls.data['orb_params']
            cls.sgbmParams = cls.data['sgbm_params']
            cls.filepaths = cls.data['file_paths']
            cls.hardwarePorts = cls.data['hardware_ports']
        except FileNotFoundError:
            raise FileNotFoundError("Cannot read config file")

    @classmethod
    def getConfig(cls) -> Dict:
        return cls.data

    @classmethod
    def getIterationConstantsDict(cls) -> Dict:
        return cls.iterationConstants

    @classmethod
    def getCameraPortsDict(cls) -> Dict:
        return cls.cameraPorts

    @classmethod
    def getCameraPorts(cls) -> (int, int):
        return cls.cameraPorts['leftPort'], cls.cameraPorts['rightPort']

    @classmethod
    def getOrbParamsDict(cls) -> Dict:
        return cls.orbParams

    @classmethod
    def getSBGMParamsDict(cls) -> Dict:
        return cls.sgbmParams

    @classmethod
    def getFilepathsDict(cls) -> Dict:
        return cls.filepaths

    @classmethod
    def getHardwarePortsDict(cls) -> Dict:
        return cls.hardwarePorts

