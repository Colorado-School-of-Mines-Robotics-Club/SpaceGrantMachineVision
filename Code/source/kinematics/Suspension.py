from typing import Union
from .elements.FourBar import FourBar


class Suspension(FourBar):
    def __init__(self, length: float, height: float):
        super().__init__(length, height)
