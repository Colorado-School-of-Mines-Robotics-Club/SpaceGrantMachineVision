from .QueuePipe import QueuePipe
import multiprocessing
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
        self.process = Process(name=self.name, target=self.run, args=(self.queue.inputQ, self.queue.outputQ,
                                                                      self.actionQueue), daemon=True)
        self.stopped = False

    def run(self, inputQ, outputQ, actionQ):
        # build the static arguments
        qPipe = QueuePipe(inputQ, outputQ)
        if self.staticObjBuilder is not None:
            staticObjects = self.staticObjBuilder(self.staticObjBuilderArgs)
            targetArgs = (qPipe,) + tuple(staticObjects) + self.args
        else:
            targetArgs = (qPipe,) + self.args
        # define signal catches
        self.handleSignals()
        # run the loop for the target function
        while not self.stopped:
            self.parseActionQueue(actionQ)
            if self.stopped:
                break
            outputs = self.target(targetArgs)
            self.putOutputs(outputs)

    def parseActionQueue(self, actionQueue: Union[Queue, None] = None):
        actionQ = self.actionQueue if actionQueue is None else actionQueue
        if not actionQ.empty():
            action = actionQ.get()
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
            self.stop()
            self.process.join(timeout=timeout)
        else:
            self.stop()
            self.process.join()

    def terminate(self):
        if self.process.is_alive():
            self.process.terminate()

    def kill(self):
        if self.process.is_alive():
            self.process.kill()

    def handleSignals(self):
        signal.signal(signal.SIGINT, self.handle_sigint)

    def handle_sigint(self, sig, frame):
        self.stop()

    # Queue functionality and interactivity
    def putInputs(self, inputs: Union[List[Any], Any]):
        self.queue.addInputs(inputs)

    def putOutputs(self, outputs: Union[List[Any], Any]):
        self.queue.addOutputs(outputs)

    def getOutputs(self, timeout: Union[float, None] = None) -> List[Any]:
        return self.queue.getOutputs(timeout=timeout)

    def getOutput(self, timeout: Union[float, None] = None) -> Any:
        return self.queue.getOutput(timeout=timeout)
