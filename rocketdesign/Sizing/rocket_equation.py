import numpy as np
from scipy.optimize import minimize, NonlinearConstraint
STANDARD_GRAV = 9.80665 # m/s/s https://en.wikipedia.org/wiki/Standard_gravity
# Size a 2 stage Rocket
def rocket_equation(I_sp, m_0, m_f):
    delta_v = I_sp * STANDARD_GRAV * np.log(m_0 / m_f)
    return delta_v



# Settings
n_engines_s1 = 9
n_engines_s2 = 1
isp_s1 = 280
engine_thrust_s1 = 100000   # N
isp_s2 = 360
engine_thrust_s2 = 100000   # N
payload = 1000
mass_fraction_s1 = 0.11
mass_fraction_s2 = 0.1

target_deltaV = 10000

deltav_loses = 1000
dry_mass_s1_guess = 6000
dry_mass_s2_guess = 2000

# Constraints 
min_thurst2_weight_s1 = 1.3
min_thurst2_weight_s2 = 0.9

def calculated_vehicle_deltav(dry_mass_s1, dry_mass_s2):

    wetmass_s1 = dry_mass_s1 / mass_fraction_s1 + dry_mass_s1
    wetmass_s2 = dry_mass_s2 / mass_fraction_s2 + dry_mass_s2

    initial_mass_stacked = wetmass_s1 + wetmass_s2 + payload
    final_stacked = wetmass_s2 + dry_mass_s1 + payload

    initial_s2 = wetmass_s2 + payload
    final_s2 = dry_mass_s2 + payload
    t2w_stacked = 9*engine_thrust_s1/ initial_mass_stacked/STANDARD_GRAV
    t2w_s2 = engine_thrust_s2/ initial_s2 /STANDARD_GRAV

    result_stacked = rocket_equation(isp_s1, initial_mass_stacked, final_stacked)

    result  = rocket_equation(isp_s2, initial_s2, final_s2)
    total_dv = result + result_stacked - deltav_loses
    return total_dv, t2w_stacked, t2w_s2
initial_guess = [dry_mass_s1_guess, dry_mass_s2_guess]

def constraint_func(z):
    _, t2w_stacked, t2w_s2 = calculated_vehicle_deltav(*z)
    return np.array([t2w_stacked, t2w_s2 ])

non_linear_constr = NonlinearConstraint(
    constraint_func, [min_thurst2_weight_s1, min_thurst2_weight_s2], [np.inf, np.inf])

def obj_func(z):
    dv_total, _, _ = calculated_vehicle_deltav(*z)
    return (target_deltaV - dv_total)**2

initial_guess = [dry_mass_s1_guess, dry_mass_s2_guess]
result = minimize(obj_func, x0=initial_guess, constraints=non_linear_constr)

print(f"Stage masses = {result.x}")
print(f"  Minimum = {result.fun}")
dv_total, t2w_stacked, t2w_s2 = calculated_vehicle_deltav(*result.x)
print(f"Change in velocity (\u0394V): {dv_total:.2f} m/s")
print(f"Stacked thrust to Weight = {t2w_stacked}")
print(f"S2 thrust to Weight = {t2w_s2}")

dry_mass_s1, dry_mass_s2 = result.x
wetmass_s1 = dry_mass_s1 / mass_fraction_s1 + dry_mass_s1
wetmass_s2 = dry_mass_s2 / mass_fraction_s2 + dry_mass_s2

initial_mass_stacked = wetmass_s1 + wetmass_s2 + payload

print(f"Total Mass = {initial_mass_stacked}")

