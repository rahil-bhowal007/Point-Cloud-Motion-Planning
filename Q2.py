from typing import Tuple
import numpy as np


def q2(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, float]:
    '''
    Localize a sphere in the point cloud. Given a point cloud as
    input, this function should locate the position and radius
    of a sphere

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting sphere center
    radius : float
        scalar radius of sphere
    '''

    ### Enter code below
    center = np.array([1,0.5,0.1])
    radius = 0.05
    num_iterations = 10000
    threshold = 0.001
    min_radius = 0.05
    max_radius = 0.11
    max_inliers = 0

    for _ in range(num_iterations):
        random_index = np.random.randint(0, len(P))
        sample_point = P[random_index]
        sample_normal = N[random_index]
        direction_vector = sample_normal - sample_point
        unit_direction = direction_vector / np.linalg.norm(direction_vector)
        random_radius = np.random.uniform(min_radius, max_radius)
        candidate_center = sample_point + random_radius * unit_direction
        point_distances = np.linalg.norm(P - candidate_center, axis=1)
        inlier_mask = ((random_radius - threshold < point_distances) & (point_distances < random_radius + threshold))
        num_inliers = np.count_nonzero(inlier_mask)

        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_center = candidate_center
            best_radius = random_radius

    print(f"Center: {best_center}, Radius: {best_radius}")
    return best_center, best_radius
