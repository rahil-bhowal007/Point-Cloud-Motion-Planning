import numpy as np
from helper_functions import *
import typing


def C5_func(q_grid: np.array, q_start: np.array, q_goal: np.array, c_path: typing.List[np.array]) -> typing.List[np.array]:
    """ Convert the path from indices of q_grid to actual robot configurations.

    Parameters
    ----------
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_start : np.array
        A 2 x 1 numpy array representing the start configuration of the robot in the format of [q1, q2].
    q_goal : np.array
        A 2 x 1 numpy array representing the goal configuration of the robot in the format of [q1, q2].
    c_path : typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using indices of q_grid.

    Returns
    -------
    typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using actual angle values.
        The first dimension is q1 and the second dimension is q2. Example: [ [q1_0 , q2_0], [q1_1, q2_1], .... ]
    """

    ### Insert your code below: ###
    q_path = np.zeros_like(c_path, dtype=float)
    for i in range(c_path.shape[0]):
        x = c_path[i, 0]
        y = c_path[i, 1]
        q_path[i, 0] = q_grid[x]
        q_path[i, 1] = q_grid[y]

    return q_path
