import numpy as np


def M1(q_min: np.array, q_max: np.array, num_samples: int) -> np.array:
    """_summary_

    Parameters
    ----------
    q_min : np.array
        1x4 numpy array of minimum angle for each joint
    q_max : np.array
        1x4 numpy array of maximum angle for each joint
    num_samples : int
        number of samples to sample

    Returns
    -------
    np.array
        num_samples x 4 numpy array of joint angles, all within joint limits
    """

    samples = np.random.uniform(q_min, q_max, size=(num_samples, len(q_min)))

    return samples
