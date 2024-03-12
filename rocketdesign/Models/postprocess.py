import pandas as pd
from scipy.spatial.transform import Rotation as R

def get_dataframe(t, states, tags):
    log_dict = {}
    log_dict["Time"] = list(t)
    for idx, tag in enumerate(tags):
        log_dict[tag] = list(states[idx,:])
    df_results = pd.DataFrame.from_dict(log_dict)

    q_IB = df_results[["q_I2B_0", "q_I2B_1", "q_I2B_2", "q_I2B_3"]].to_numpy()
    q_IB_r = q_IB[:, [1,2,3,0]]
    orent_IB = R.from_quat(q_IB_r)
    e_xyz_IB = orent_IB.as_euler('XYZ')
    df_results['e_xyz_IB_0'] = e_xyz_IB[:, 0]
    df_results['e_xyz_IB_1'] = e_xyz_IB[:, 1]
    df_results['e_xyz_IB_2'] = e_xyz_IB[:, 2]
    return df_results