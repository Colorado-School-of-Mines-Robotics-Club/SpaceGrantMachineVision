# Built in python libs
from typing import List

# Additional libs

# Custom imports


# takes an array of times and returns the average over a size
def getAvgTimeArr(arr: List, size: int) -> float:
    return round((sum(arr) / size) * 1000, 1)
