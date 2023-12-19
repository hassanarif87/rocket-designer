

import numpy as np
def skew_sym_mat(vec):
    """
    this function returns a numpy array with the skew symmetric cross product matrix for vector.
    the skew symmetric cross product matrix is defined such that
    np.cross(a, b) = np.dot(skew(a), b)

    :param x: An array like vector to create the skew symmetric cross product matrix for
    :return: A numpy array of the skew symmetric cross product vector
    """

    return np.array([[0, -vec[2], vec[1]], 
                     [vec[2], 0, -vec[0]], 
                     [-vec[1], vec[0], 0]])

def identiy_mul(x):
    return np.identity(3) * x

def quat_conj(q):
    return np.array([q[0] , -q[1], -q[2], -q[3]])
                    
# q · p = qm(q, p) = Q(q)p = Q¯(p)q (106)
# p · q = qm(p, q) = Q(p)q = Q¯(q)p, (107)
def quat_mat(q):
    """Eq 109 pg 14
        https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
    """
    return np.array([
        [q[0] , -q[1], -q[2], -q[3]],
        [q[1] , q[0], q[3], -q[2]],
        [q[2] , -q[3], q[0], q[1]],
        [q[3] , q[2], -q[1], q[0]]
    ])

def quat_mat_conj(q):
    """Eq 111 pg 14
        https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
    """
    return np.array([
        [q[0] , -q[1], -q[2], -q[3]],
        [q[1] , q[0], -q[3], q[2]],
        [q[2] , q[3], q[0], -q[1]],
        [q[3] , -q[2], q[1], q[0]]
    ])

def quat_rate_mat(q):
    """Eq 150 pg 16
        https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
    """
    return np.array([
        [-q[1] , q[0], -q[3], q[2]],
        [-q[2] , q[3], q[0], -q[1]],
        [-q[3] , -q[2], q[1], q[0]]
    ])

def quat_rate_mat_conj(q):
    """Eq 151 pg 16
        https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf
    """
    return np.array([
        [-q[1] , q[0], q[3], -q[2]],
        [-q[2] , -q[3], q[0], q[1]],
        [-q[3] , q[2], -q[1], q[0]]
    ])
