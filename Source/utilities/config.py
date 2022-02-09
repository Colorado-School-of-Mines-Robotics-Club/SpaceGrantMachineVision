# Built in python libs
import sys
import os
from typing import Dict
import json

# Additional libs
import cv2
import numpy as np

# Custom imports
try:
    from logger.logger import Logger
except ImportError:
    from Source.logger.logger import Logger


def readConfigFile(file="config.json") -> Dict:
    try:
        with open(file, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        raise FileNotFoundError(f"Couldn't open {file}")
