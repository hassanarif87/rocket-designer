import numpy as np
from dataclasses import dataclass, field
from typing import Any

@dataclass
class State3dofRocket:
    """Class discribing the state of a Body() in an inertial refframe
    The angular velocity is discribed in the body refframe of the Body()
    """

    pos: np.array  # Position w.r.t a the base integration refframe, this frame is assumed to be inertia
    vel_I: np.array  # Velocity in the base inertial refframe
    prop_mass: float
    @property
    def vector(self):
        """Get state vector
        Returns:
            state_vec: return the state vector as an np array
        """
        return np.hstack((self.pos, self.vel_I, self.mass))

    @classmethod
    def zero_states(cls):
        return cls(
            pos=np.zeros([3]),
            vel_I=np.zeros([3]),
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




@dataclass
class Body3Dof:
    """Class discribing a 3 Dof body with changing mass in inertial space"""

    dry_mass: float # dry masss
    force_obj_list: list[Any] = field(default_factory=list)
    dcm_IrB0 : np.array 
    g_func: callable
    attitude_fun: callable

    @classmethod
    def default(cls):
        return cls(
            mass=100.0,
        )

    def grav(self, r_vec):
        """Get gravity
        Args:
            r_vec : Position vector w.r.t central body
        Returns:
            grav_vec: Gravity vector at position
        """

        return self.g_func(r_vec)

    def add_force_object(self, obj):
        self.force_obj_list.append(obj)

    def force_collector(self, state):
        """Iterates through force objects attached to the Body and
        collects and calculates the forces and moments about the Bodys
        center of mass

        Args:
            state : Current state vector of the Body
        Returns:
            forces : Forces in the body frame
        """
        sum_force_body = np.array([0,0,0]) #np.zeros(3)

        for force_obj in self.force_obj_list:
            sum_force_body += force_obj.force
        return sum_force_body

    def derivative(self, t, state):
        """
        Calcualte derivative at current state
        Args
            t: time. needed for integrator
            state: Current state vector of the Body
        Returns:
            d_state: Derivative of the state vector
        """
        d_pos = state[3:6]
        mass = state[6]
        dcm_IrB = self.attitude_fun(self, t, state)
        forces_B = self.forcetorque_collector(state)

        forces_I  = dcm_IrB @ forces_B
        # Translational equations of motion in the inertial coordinate system
        non_grav_accel = forces_I / (self.dry_mass + mass)
        d_vel_I = non_grav_accel + self.grav(state[0:3])
        d_mass = self.propulsion.mdot
        return np.hstack((d_pos, d_vel_I, d_mass))


def hit_ground(t, state):
    """Termination event on height

    Args:
        t : time, needed for the integrator
        state : Current state vector of the Body

    Returns:
        height: Height above the ground
    """
    return state[2]


hit_ground.terminal = True
hit_ground.direction = -1  # Terminate when value crosses zero

if __name__ == "__main__":
    state = State3dofRocket.zero_states()
    body = Body3Dof.default()
    state.vel_I = np.array([10,10,25])
    body.derivative(0, state.vector)
