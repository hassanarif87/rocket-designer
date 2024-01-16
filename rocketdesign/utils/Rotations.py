import numpy as np
import numpy.typing as npt

def Tx(t):
    return np.array([[1, 0, 0], [0, np.cos(t), np.sin(t)], [0, -np.sin(t), np.cos(t)]])


def Ty(t):
    return np.array([[np.cos(t), 0, -np.sin(t)], [0, 1, 0], [np.sin(t), 0, np.cos(t)]])


def Tz(t):
    return np.array([[np.cos(t), np.sin(t), 0], [-np.sin(t), np.cos(t), 0], [0, 0, 1]])


def mat_from_quat(q: npt.ArrayLike)-> npt.ArrayLike:
    """
    Convert a Unit quaternion representing a rotation to a  Transformation matrix
    https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf pg. 15 eq 125

    Args:
        q : Unit Quaternion
    Returns:
        mat : 3x3 direct cosine matrix
    """
    q2 = q**2
    q1q2 = q[1]*q[2]
    q0q3 = q[0]*q[3]
    q1q3 = q[1]*q[3] 
    q0q2 = q[0]*q[2]
    q2q3 = q[2]*q[3] 
    q0q1 = q[0]*q[1]
    return np.array([
        [q2[0] + q2[1] - q2[2] - q2[3], 2*(q1q2 + q0q3), 2*(q1q3- q0q2)],
        [2*(q1q2 - q0q3), q2[0] - q2[1] + q2[2] - q2[3], 2*(q2q3+ q0q1)],
        [2*(q1q3+ q0q2), 2*(q2q3- q0q1), q2[0] - q2[1] - q2[2] + q2[3]]
             ])

def quat_from_mat(mat: npt.ArrayLike) -> npt.ArrayLike:
    """
    DCM to Unit quaternion 
    https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf pg. 15 eq 142

    Args:
        mat (2d_array_like): 3x3 direct cosine matrix
    Returns:
        q (array_like): Unit Quaternion
    """
    return 0.5 * np.array([
        np.sqrt(1 + mat[0][0] + mat[1][1] + mat[2][2]),
        (mat[1][2] - mat[2][1])/np.sqrt(1 + mat[0][0] + mat[1][1] + mat[2][2]),
        (mat[2][0] - mat[0][2])/np.sqrt(1 + mat[0][0] + mat[1][1] + mat[2][2]),
        (mat[0][1] - mat[1][0])/np.sqrt(1 + mat[0][0] + mat[1][1] + mat[2][2])
    ])
