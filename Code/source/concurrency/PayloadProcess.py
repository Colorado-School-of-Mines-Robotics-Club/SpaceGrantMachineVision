from .QueuePipe import QueuePipe
from multiprocessing import Process, Queue
from typing import List, Tuple, Union, Any
from collections.abc import Callable
import time


class PayloadProcess:
    def __init__(self, payload: Tuple[str, Callable, Tuple, Callable, Tuple, Union[float, None]]):
        self.name, self.target, self.args, self.staticObjBuilder, self.staticObjBuilderArgs, self.qTimeout = payload
        self.queue = QueuePipe()
        self.actionQueue = Queue()
        if self.qTimeout is not None:
            self.queue = QueuePipe(timeout=self.qTimeout)
        self.process = Process(name=self.name, target=self.run, args=(), daemon=True)
        self.stopped = False

    def run(self):
        # build the static arguments
        if len(self.staticObjBuilderArgs) == 0:
            staticObjects = self.staticObjBuilder()
        else:
            staticObjects = self.staticObjBuilder(self.staticObjBuilderArgs)
        targetArgs = (self.queue, ) + tuple(staticObjects) + self.args
        # run the loop for the target function
        while not self.stopped:
            self.parseActionQueue()
            self.putOutputs(self.target(targetArgs))

    def parseActionQueue(self):
        if not self.actionQueue.empty():
            action = self.actionQueue.get()
            if isinstance(action, float) or isinstance(action, int):
                time.sleep(action)
            elif isinstance(action, str):
                if action == 'stop':
                    self.stopped = True

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

    def join(self, timeout: Union[float, None] = None):
        if timeout is not None:
            self.process.join(timeout=timeout)
        else:
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
