import numpy as np
from helper_functions import *


def C7_func(cspace: np.array) -> np.array:
    """Pad the configuration space by one grid cell.

    Parameters
    ----------
    cspace : np.array
        The origianl configuration space of the robot.

    Returns
    -------
    np.array
        The padded configuration space of the robot.
    """

    ### Insert your code below: ###
    padded_cspace = cspace.copy()

    for i in range(1, cspace.shape[0] - 1):
        for j in range(1, cspace.shape[1] - 1):
            if cspace[i, j] == 0:
                if (cspace[i-1, j-1] == 1 or
                    cspace[i-1, j] == 1 or
                    cspace[i-1, j+1] == 1 or
                    cspace[i, j-1] == 1 or
                    cspace[i, j+1] == 1 or
                    cspace[i+1, j-1] == 1 or
                    cspace[i+1, j] == 1 or
                        cspace[i+1, j+1] == 1):
                    padded_cspace[i, j] = 1
                else:
                    padded_cspace[i, j] = 0

    return padded_cspace
