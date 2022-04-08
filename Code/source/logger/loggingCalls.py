# Built in python libs
import sys
from typing import Dict, Type

# Additional libs
import platform

# Custom imports
try:
    from source.utilities.Config import Config
    from .Logger import Logger
except ModuleNotFoundError:
    from Code.source.utilities.Config import Config
    from .Logger import Logger


def logSystemInfo(logger: Type[Logger]):
    # Log system information
    logger.log("SYSTEM INFORMATION:")
    logger.log(f"   Python Version: {sys.version}")
    uname = platform.uname()
    logger.log(f"   System: {uname.system}")
    logger.log(f"   Node Name: {uname.node}")
    logger.log(f"   Release: {uname.release}")
    logger.log(f"   Version: {uname.version}")
    logger.log(f"   Machine: {uname.machine}")
    logger.log(f"   Processor: {uname.processor}")


def logArguments(logger: Type[Logger], args: Dict):
    logger.log("ARGUMENTS:")
    for arg, val in args.items():
        logger.log(f"    {arg}: {val}")


def logConfiguration(logger: Type[Logger], config=None, spacing="    ", first=True, level=1):
    if config is None:
        Config.init()
        config = Config.getConfig()
    if first:
        logger.log("CONFIGURATION:")
    for key, val in config.items():
        if isinstance(val, dict):
            logger.log(f"{spacing*level}{key}")
            logConfiguration(logger, config=val, spacing=spacing, first=False, level=level+1)
        else:
            logger.log(f"{spacing*level}{key}: {val}")
