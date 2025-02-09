import numpy as np


class Point:

    def __init__(self, x: float, y: float):
        self.x: float = x
        self.y: float = y


    def distance(self) -> float:
        return np.sqrt(np.power(self.x, 2) + np.power(self.y, 2))
