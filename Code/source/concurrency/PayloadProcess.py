from .QueuePipe import QueuePipe
from multiprocessing import Process
from typing import List, Tuple, Union, Any
from collections.abc import Callable
import time


class PayloadProcess:
    def __init__(self, payload: Tuple[str, Callable, Tuple, float]):
        self.name, self.target, self.args, self.qTimeout, self.sleepTime = payload
        self.queue = QueuePipe(timeout=self.qTimeout)
        self.args = (self.queue,) + self.args
        self.process = Process(name=self.name, target=self.run, args=self.args, daemon=True)
        self.stopped = False
        self.sleeping = False

    def run(self):
        while not self.stopped:
            if self.sleeping:
                time.sleep(self.sleepTime)
                self.sleeping = False
                continue
            self.putOutputs(self.target(self.args))

    def start(self) -> 'PayloadProcess':
        self.process.start()
        return self

    def status(self) -> Tuple[bool, bool]:
        alive = self.process.is_alive()
        running = self.stopped
        return alive, running

    def sleep(self, delay: float):
        self.sleeping = True
        self.sleepTime = delay

    def stop(self):
        self.stopped = True

    def join(self):
        self.process.join()

    def terminate(self):
        self.process.terminate()

    def kill(self):
        self.process.kill()

    # Queue functionality and interactivity
    def putInputs(self, inputs: Union[List[Any], Any]):
        self.queue.addInputs(inputs)

    def getOutputs(self) -> List[Any]:
        return self.queue.getOutputs()

    def getOutput(self) -> Any:
        return self.queue.getOutput()
