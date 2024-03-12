import numpy as np

class SimpleAeroModel:
    def __init__(self, Fd):
        self.Fd = Fd
        self.atm
        self.wind 

    def update(self, t, state):
        # v_eci
        # wind_ned
        # ned2ecef2eci
        # wind_eci
        rho, c =self.atm(state)
        v_rel_atm_eci =  v_eci - wind_eci
        mach  =  v_mag_rel_atm_eci / c
        if mach > 1 :
            0.25/mach
        else:
            C_db = 0.13 + 1.13*mach*mach 
        #  D= 12 CDS vair vair
        force_drag = 0.5 * rho *Cd * v_rel_atm_eci * np.norm(v_rel_atm_eci) 
        return np.array([0,0,0])
