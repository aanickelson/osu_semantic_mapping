
import numpy as np


class PlaneObject:
    def __init__(self):
        self.coefficients = np.zeros(4)
        self.inliers = []
