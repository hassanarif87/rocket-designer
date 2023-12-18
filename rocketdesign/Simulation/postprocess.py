import pandas as pd

def get_dataframe(t, states, tags):
    log_dict = {}
    log_dict["Time"] = list(t)
    for idx, tag in enumerate(tags):
        log_dict[tag] = list(states[idx,:])
    return pd.DataFrame.from_dict(log_dict)
