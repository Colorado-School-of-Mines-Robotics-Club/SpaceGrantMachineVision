from typing import List, Tuple, Dict, Union, Any
from collections.abc import Callable
from .PayloadProcess import PayloadProcess


class PayloadManager:
    payloads: Dict = dict()

    @classmethod
    def init(cls, payloads: List[Tuple[str, Callable, Tuple, Callable, Tuple, Union[float, None]]]):
        for payload in payloads:
            cls.payloads[payload[0]] = PayloadProcess(payload)

    @classmethod
    def initStart(cls, payloads: List[Tuple[str, Callable, Tuple, Callable, Tuple, Union[float, None]]]):
        cls.init(payloads)
        cls.startAll()

    # Methods for managing the processes
    @classmethod
    def start(cls, name: str):
        cls.payloads[name] = cls.payloads[name].start()

    @classmethod
    def startAll(cls):
        for name in cls.payloads:
            cls.start(name)

    @classmethod
    def stop(cls, name: str):
        cls.payloads[name].stop()

    @classmethod
    def stopAll(cls):
        for name in cls.payloads:
            cls.stop(name)

    @classmethod
    def status(cls, name: str) -> Tuple[bool, bool]:
        return cls.payloads[name].status()

    @classmethod
    def statusAll(cls) -> List[Tuple[bool, bool]]:
        statuses = list()
        for name in cls.payloads:
            statuses.append(cls.status(name))
        return statuses

    @classmethod
    def sleep(cls, name: str, delay: float):
        cls.payloads[name].sleep(delay)

    @classmethod
    def sleepAll(cls, delay: float):
        for name in cls.payloads:
            cls.sleep(name, delay)

    @classmethod
    def join(cls, name: str, timeout: Union[float, None] = None):
        if timeout is not None:
            cls.payloads[name].join(timeout=timeout)
        else:
            cls.payloads[name].join()

    @classmethod
    def joinAll(cls, timeout: Union[float, None] = None):
        for name in cls.payloads:
            cls.join(name, timeout)

    @classmethod
    def terminate(cls, name: str):
        cls.payloads[name].terminate()

    @classmethod
    def terminateAll(cls):
        for name in cls.payloads:
            cls.terminate(name)

    @classmethod
    def kill(cls, name: str):
        cls.payloads[name].kill()

    @classmethod
    def killAll(cls):
        for name in cls.payloads:
            cls.kill(name)

    @classmethod
    def close(cls, name: str, timeout: Union[float, None] = None):
        cls.stop(name)
        cls.join(name, timeout)
        cls.terminate(name)

    @classmethod
    def closeAll(cls, timeout: Union[float, None] = None):
        for name in cls.payloads:
            cls.close(name, timeout)

    # methods for adding and getting data from the process queues
    @classmethod
    def addInputs(cls, name: str, inputs: Union[List[Any], Any]):
        cls.payloads[name].putInputs(inputs)

    @classmethod
    def getOutputs(cls, name: str) -> List[Any]:
        return cls.payloads[name].getOutputs()

    @classmethod
    def getOutput(cls, name: str) -> Any:
        return cls.payloads[name].getOutput()
