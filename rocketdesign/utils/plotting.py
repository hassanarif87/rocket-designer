import plotly.graph_objects as go
#from plotly.graph_objects.Figure import add_trace
import numpy as np

def plot_axis(axis=None, id=1, line_type='solid', fig=None):
    if axis is None:
        axis = np.identity(3)
    if fig is None:
        fig = go.Figure()
    axis_name = ['x'+str(id), 'y'+str(id), 'z'+str(id)]
    colors= ['red', 'blue', 'green']
    for idx, name in enumerate(axis_name):
        fig.add_trace(
            go.Scatter3d(
                # Each row of the matrix represent the basis vector in the base frame
                x=[0,axis[idx,0]] , y=[0,axis[idx,1]], z=[0,axis[idx,2]], name=name, 
                line=dict(color=colors[idx], width=4, dash=line_type),
            )
        )
    return fig