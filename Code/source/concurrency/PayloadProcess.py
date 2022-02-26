from .QueuePipe import QueuePipe
from multiprocessing import Process, Queue
from typing import List, Tuple, Union, Any
from collections.abc import Callable
import time


class PayloadProcess:
    def __init__(self, payload: Tuple[str, Callable, Tuple, float]):
        self.name, self.target, self.args, self.qTimeout = payload
        self.queue = QueuePipe(timeout=self.qTimeout)
        self.args = (self.queue,) + self.args
        self.process = Process(name=self.name, target=self.run, args=(), daemon=True)
        self.stopped = False
        self.actionQueue = Queue()

    def run(self):
        while not self.stopped:
            if not self.actionQueue.empty():
                action = self.actionQueue.get()
                if isinstance(action, float) or isinstance(action, int):
                    time.sleep(action)
                    continue
                if isinstance(action, str):
                    if action == 'stop':
                        self.stopped = True
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
        self.actionQueue.put(delay)

    def stop(self):
        self.actionQueue.put('stop')

    def join(self):
        self.process.join()

    def terminate(self):
        self.process.terminate()

    def kill(self):
        self.process.kill()

    # Queue functionality and interactivity
    def putInputs(self, inputs: Union[List[Any], Any]):
        self.queue.addInputs(inputs)

    def putOutputs(self, outputs: Union[List[Any], Any]):
        self.queue.addOutputs(outputs)

    def getOutputs(self) -> List[Any]:
        return self.queue.getOutputs()

    def getOutput(self) -> Any:
        return self.queue.getOutput()
