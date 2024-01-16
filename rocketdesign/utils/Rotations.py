import numpy as np


def Tx(t):
    return np.array([[1, 0, 0], [0, np.cos(t), np.sin(t)], [0, -np.sin(t), np.cos(t)]])


def Ty(t):
    return np.array([[np.cos(t), 0, -np.sin(t)], [0, 1, 0], [np.sin(t), 0, np.cos(t)]])


def Tz(t):
    return np.array([[np.cos(t), np.sin(t), 0], [-np.sin(t), np.cos(t), 0], [0, 0, 1]])


def q_to_T(q):
    """
    Convert a Unit quaternion representing a rotation to a  Transformation matrix
    https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf pg. 15 eq 125

    Args:
        q (array_like): Unit Quaternion
    """
    return np.array([[q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2, 2*(q[1]*q[2] + q[0]*q[3]), 2*(q[1]*q[3] - q[0]*q[2])],
             [2*(q[1]*q[2] - q[0]*q[3]), q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2, 2*(q[2]*q[3] + q[0]*q[1])],
             [2*(q[1]*q[3] + q[0]*q[2]), 2*(q[2]*q[3] - q[0]*q[1]), q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2]])