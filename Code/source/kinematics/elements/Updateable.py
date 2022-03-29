import time


class Updateable:
    def __init__(self):
        self.lastUpdated = time.perf_counter()

    def update(self):
        self.lastUpdated = time.perf_counter()
