import numpy as np
from dataclasses import dataclass


@dataclass
class State:
    """Class discribing the state of a Body() in an inertial refframe
    The angular velocity is discribed in the body refframe of the Body()    
    """

    pos: np.array # Position w.r.t a the base integration refframe, this frame is assumed tobe inertia
    vel_I: np.array # Velocity in the base inertial refframe
    q_I2B: np.array # Quaternion orientation from inertial to body 
    ang_vel_B: np.array # Angular velocity in the body refframe

    @property
    def vector(self):
        """Get state vector
        Returns:
            state_vec: return the state vector as an np array
        """
        return np.hstack((self.pos, self.vel_I, self.q_B2I, self.ang_vel_B))

    @classmethod
    def zero_states(cls):
        return cls(
            pos=np.zeros([3]),
            vel_I=np.zeros([3]),
            q_B2I=np.array([1, 0, 0, 0]),
            ang_vel_B=np.zeros([3]),
        )

    @property
    def get_tags(self):
        """Gets state name list
        Returns:
            tag_list: Expanded string name of all the states in the vector
        """
        tag_list = []
        for key, value in self.__dict__.items():
            for i, _ in enumerate(value):
                name = f"{key}_{i}"
                tag_list.append(name)
        return tag_list


def get_gravaty(r_vec):
    """Get gravity
    Args:
        r_vec : Position vector w.r.t central body
    Returns:
        grav_vec: Gravity vector at position
    """
    return np.array([0, 0, -9.81])


class Body:
    """Class discribing a body in inertial space"""

    mass = 1.0
    inertia = np.zeros([3, 3])
    com = np.zeros([3])

    @staticmethod
    def grav(r_vec):
        return get_gravaty(r_vec)

    def forces(self, q_B2I):
        """Sums all forces in the body frame and convertes them to the I frame

        Args:
            state : Current state vector of the Body
        Returns:
            forces_I : Forces in the inerital frame
        """
        return np.zeros([3])

    def derivative(self, t, state):
        d_pos = state[3:6]
        non_grav_accel = self.forces(state[6:10]) / self.mass
        d_vel_I = non_grav_accel + self.grav(state[0:3])
        d_q_B2I = np.array([0, 0, 0, 0])
        d_ang_vel_B = np.zeros([3])
        return np.hstack((d_pos, d_vel_I, d_q_B2I, d_ang_vel_B))


def hit_ground(t, state):
    """Termination event on height

    Args:
        t : time, needeo for the integrator
        state : Current state vector of the Body

    Returns:
        height: Height above the ground
    """
    return state[2]


hit_ground.terminal = True
hit_ground.direction = -1  # Terminate when value crosses zero
