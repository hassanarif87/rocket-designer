import numpy as np
from dataclasses import dataclass
from rocketdesign.utils.Quaternions import quat_rate_mat_conj


@dataclass
class State:
    """Class discribing the state of a Body() in an inertial refframe
    The angular velocity is discribed in the body refframe of the Body()
    """

    pos: np.array  # Position w.r.t a the base integration refframe, this frame is assumed tobe inertia
    vel_I: np.array  # Velocity in the base inertial refframe
    q_I2B: np.array  # Quaternion orientation from inertial to body
    ang_vel_B: np.array  # Angular velocity in the body refframe

    @property
    def vector(self):
        """Get state vector
        Returns:
            state_vec: return the state vector as an np array
        """
        return np.hstack((self.pos, self.vel_I, self.q_I2B, self.ang_vel_B))

    @classmethod
    def zero_states(cls):
        return cls(
            pos=np.zeros([3]),
            vel_I=np.zeros([3]),
            q_I2B=np.array([1, 0, 0, 0]),
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

@dataclass
class Body:
    """Class discribing a body in inertial space"""

    mass: float
    moi: np.array
    com: np.array
    moi_inv: np.array
    @classmethod
    def default(cls):
        return cls(
            mass=1.0,
            moi=np.identity(3),
            com=np.array([0, 0, 0]),
            moi_inv=np.identity(3)
        )
    @staticmethod
    def grav(r_vec):
        return get_gravaty(r_vec)

    def forcetorque(self, q_I2B):
        """Sums all forces in the body frame and convertes them to the I frame

        Args:
            state : Current state vector of the Body
        Returns:
            forces_I : Forces in the inerital frame
        """
        return np.zeros([3]), np.array([-1,0,0])

    def derivative(self, t, state):
        d_pos = state[3:6]
        q_I2B = state[6:10]
        ang_vel_B = state[10:]
        # Translational equations of motion in the inertial coordinate system
        forces_I, torques = self.forcetorque(q_I2B)
        non_grav_accel = forces_I / self.mass
        d_vel_I = non_grav_accel + self.grav(state[0:3])

        # Rotational equations of motion in the body coordinate system
        moi_omega = np.matmul(self.moi, ang_vel_B)
        d_q_I2B = 0.5 * np.matmul(quat_rate_mat_conj(q_I2B).T, ang_vel_B)
        ang_momentum_B = torques - np.cross(ang_vel_B, moi_omega)
        d_ang_vel_B = np.matmul(self.moi_inv, ang_momentum_B)
        return np.hstack((d_pos, d_vel_I, d_q_I2B, d_ang_vel_B))


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

if __name__ == "__main__":
    state = State.zero_states()
    body = Body.default()
    state.vel_I = np.array([10,10,25])
    body.derivative(0, state.vector)
