import numpy as np
from pprint import pformat


class Vector:

    def __init__(self, x=0.0, y=0.0):
        self.data = np.array([x, y])

    def __repr__(self):
        return pformat((self.x, self.y))

    @property
    def x(self):
        return self.data[0]

    @x.setter
    def x(self, value):
        self.data[0] = value

    @property
    def y(self):
        return self.data[1]

    @y.setter
    def y(self, value):
        self.data[1] = value

    @staticmethod
    def distance(v0, v1):
        return np.linalg.norm(v0.data - v1.data)

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)
