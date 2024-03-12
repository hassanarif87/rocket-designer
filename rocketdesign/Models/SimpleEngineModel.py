import numpy as np

class SimpleEngineModel:
    def __init__(self, thrust, isp):
        self.thrust = thrust
        self.isp = isp

    def update(self, state):

        return np.array([self.thrust,0,0])