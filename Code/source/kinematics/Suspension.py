from typing import Union
from .elements.ParallelogramFourBar import ParallelogramFourBar


class Suspension(ParallelogramFourBar):
    def __init__(self, length: float, height: float):
        super().__init__(length, height)
