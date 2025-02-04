import numpy as np


def M0() -> np.array:
    """ Test different configuration of the robot. 
    Play around with different joint angles to see how the robot moves

    Returns
    -------
    np.array
        1x4 numpy array of joint angles
    """
    return np.array([0, -np.pi/4, 0, -np.pi/4])
