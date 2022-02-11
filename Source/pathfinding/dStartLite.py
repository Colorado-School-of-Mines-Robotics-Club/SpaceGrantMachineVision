# Built in python libs
import sys
import os
from typing import Dict, List

# Additional libs
import cv2
import numpy as np

# Custom imports
try:
    from logger.logger import Logger
    from utilities.Config import Config
except ImportError:
    from Source.logger.Logger import Logger
    from Source.utilities.Config import Config
