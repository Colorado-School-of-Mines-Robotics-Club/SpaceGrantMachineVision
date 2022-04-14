import queue
from multiprocessing import Queue
from typing import Any, List, Union
import numpy as np


class QueuePipe:
    def __init__(self, inputQueue: Union[Queue, None] = None, outputQueue: Union[Queue, None] = None, timeout=None):
        self.inputQ = Queue() if inputQueue is None else inputQueue
        self.outputQ = Queue() if outputQueue is None else outputQueue
        self.totalInputs = 0
        self.totalOutputs = 0
        self.currentInputs = 0
        self.currentOutputs = 0
        self.timeout = timeout

    # acquires all items from either queue
    def getFromQueue(self, output=False, timeout: Union[float, None] = None) -> List[Any]:
        targetQueue = self.inputQ if not output else self.outputQ
        targetEmpty = self.inputEmpty if not output else self.outputEmpty
        items = list()
        while not targetEmpty():
            try:
                items.append(targetQueue.get(timeout=timeout if timeout is not None else self.timeout))
                if output:
                    self.currentOutputs -= 1
                else:
                    self.currentInputs -= 1
            except queue.Empty:
                break
        return items

    # adds items to either queue
    def addToQueue(self, items: Union[List[Any], Any], output=False):
        try:
            if isinstance(items, np.ndarray) or isinstance(items, tuple):
                if output:
                    self.outputQ.put(items)
                else:
                    self.inputQ.put(items)
                return
            for item in items:
                if output:
                    self.outputQ.put(item)
                else:
                    self.inputQ.put(item)
            length = len(items)
            if output:
                self.currentOutputs += length
                self.totalOutputs += length
            else:
                self.currentInputs += length
                self.totalInputs += length
        except TypeError:
            if output:
                self.outputQ.put(items)
                self.currentOutputs += 1
                self.totalOutputs += 1
            else:
                self.inputQ.put(items)
                self.currentInputs += 1
                self.totalInputs += 1

    # acquires items from input queue and returns
    def getInputs(self) -> List[Any]:
        return self.getFromQueue(output=False)

    # acquires just the first item from the input queue
    def getInput(self) -> Any:
        try:
            item = self.inputQ.get(timeout=self.timeout)
            self.currentInputs -= 1
            return item
        except queue.Empty:
            return None

    # acquires items from output queue and returns
    def getOutputs(self, timeout: Union[float, None] = None) -> List[Any]:
        return self.getFromQueue(output=True, timeout=timeout)

    # gets the first item from the output queue
    def getOutput(self, timeout: Union[float, None] = None) -> Any:
        try:
            item = self.outputQ.get(timeout=self.timeout if timeout is None else timeout)
            self.currentOutputs -= 1
            return item
        except queue.Empty:
            return None

    # puts items into the input queue
    def addInputs(self, inputs: Union[List[Any], Any]):
        self.addToQueue(inputs, output=False)

    # puts items into output queue
    def addOutputs(self, outputs: Union[List[Any], Any]):
        self.addToQueue(outputs, output=True)

    # puts an object into the QPipes input queue
    def put(self, inputs: Union[List[Any], Any]):
        self.addInputs(inputs)

    # gets an object out of the QPipes output queue
    def get(self) -> Any:
        return self.getOutput()

    # returns if the QPipes input queue is empty
    def inputEmpty(self):
        return self.inputQ.empty()

    # returns if the QPipes output queue is empty
    def outputEmpty(self):
        return self.outputQ.empty()

    # returns if both the input and output queues are empty
    def empty(self, condition='and'):
        if condition == 'and':
            return self.inputEmpty() and self.outputEmpty()
        elif condition == 'or':
            return self.inputEmpty() or self.outputEmpty()
        else:
            raise Exception(f"Unrecognized condition type in QPipe.empty(): {condition}")

    # Getters for the counter member variables
    def getTotalInputs(self) -> int:
        return self.totalInputs

    def getTotalOutputs(self) -> int:
        return self.totalOutputs

    def getCurrentInputs(self) -> int:
        return self.currentInputs

    def getCurrentOutputs(self) -> int:
        return self.currentOutputs

    def getTotalInputOutputRatio(self) -> float:
        try:
            return self.getTotalInputs() / self.getTotalOutputs()
        except ZeroDivisionError:
            return float('inf')

    def getCurrentInputOutputRatio(self) -> float:
        try:
            return self.getCurrentInputs() / self.getCurrentOutputs()
        except ZeroDivisionError:
            return float('inf')
