from .QueuePipe import QueuePipe
from multiprocessing import Process, Queue
from typing import List, Tuple, Union, Any
from collections.abc import Callable
import time
import signal


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
        staticObjects = self.staticObjBuilder(self.staticObjBuilderArgs)
        targetArgs = (self.queue, ) + tuple(staticObjects) + self.args
        # run the loop for the target function
        while not self.stopped:
            self.handleSignals()
            self.parseActionQueue()
            if self.stopped:
                break
            self.putOutputs(self.target(targetArgs))

    def parseActionQueue(self):
        if not self.actionQueue.empty():
            print("SOMETHING IN THE QUUEE")
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
        if not self.process.is_alive():
            return
        if timeout is not None:
            self.process.join(timeout=timeout)
        else:
            self.process.join()

    def terminate(self):
        if self.process.is_alive():
            self.process.terminate()

    def kill(self):
        if self.process.is_alive():
            self.process.kill()

    def handleSignals(self):
        signal.signal(signal.SIGINT, self.handle_sigint)
        signal.signal(signal.SIGTERM, self.handle_sigterm)

    def handle_sigint(self, sig, frame):
        self.stopped = True

    def handle_sigterm(self, sig, frame):
        self.stopped = True

    # Queue functionality and interactivity
    def putInputs(self, inputs: Union[List[Any], Any]):
        self.queue.addInputs(inputs)

    def putOutputs(self, outputs: Union[List[Any], Any]):
        self.queue.addOutputs(outputs)

    def getOutputs(self) -> List[Any]:
        return self.queue.getOutputs()

    def getOutput(self) -> Any:
        return self.queue.getOutput()
