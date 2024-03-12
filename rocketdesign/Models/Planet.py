
import numpy as np

class Earth:
    def __init__(self):
        # https://en.wikipedia.org/wiki/Geopotential_model
        self.J2 = 1082.6e-6
         #"wgs84": {"name": "WGS-84 (1984)", "a": 6378137.0, "b": 6356752.31424518},
        self.omega_I = np.array([0,0,7.292115e-5])
    def specific_grav(self, coord_ecef):
        x,y,z = coord_ecef
        x2 = x*x
        y2 = y*y
        z2 = z*z
        r = np.sqrt(x2+y2+z2)
        r7 = r**7
        x2_y2 = x2 + y2
        gx = self.J2 * x /r7 * ( 6 * z2 - 3/2*(x2_y2))
        gy = self.J2 * y /r7 * ( 6 * z2 - 3/2*(x2_y2))
        gz = self.J2 * z /r7 * ( 3 * z2 - 9/2*(x2_y2))
        return np.array([gx, gy,gz])