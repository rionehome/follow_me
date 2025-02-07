import numpy as np


class Point:

    def __init__(self, x: float, y: float):
        self.x: float = x
        self.y: float = y


    def distance(self) -> float:
        """Returns distance from the origin."""
        return np.hypot(self.x, self.y)


    @staticmethod
    def hypot(point1: Point, point2: Point) -> float:
        """Returns distance between two Points."""
        return np.hypot(point2.x - point1.x, point2.y - point1.y)


class PointData:

    def __init__(self, index: int, point: Point, existence_rate: float):
        self.index:            int = index
        self.point:          Point = point
        self.existence_rate: float = existence_rate
