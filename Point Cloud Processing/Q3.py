from typing import Tuple
import numpy as np


def q3(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    '''
    Localize a cylinder in the point cloud. Given a point cloud as
    input, this function should locate the position, orientation,
    and radius of the cylinder

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting 100 points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting cylinder center
    axis : np.ndarray
        array of shape (3,) pointing along cylinder axis
    radius : float
        scalar radius of cylinder
    '''


    ### Enter code below
    center = np.array([1,0.5,0.1])
    axis = np.array([0,0,1])
    radius = 0.05
    max_iterations = 10000
    inlier_threshold = 0.01
    num_points = P.shape[0]
    max_inliers_count = 0
    best_cyl_center = np.zeros(3)
    best_cyl_axis = np.zeros(3)
    best_cyl_radius = 0.0

    for _ in range(max_iterations):
        sample_indices = np.random.choice(num_points, 2, replace=False)
        sampled_points = P[sample_indices]
        sampled_normals = N[sample_indices]
        candidate_radius = np.random.uniform(0.05, 0.10)
        axis_vector = np.cross(sampled_normals[0], sampled_normals[1])
        axis_unit_vector = axis_vector / np.linalg.norm(axis_vector)
        candidate_center = sampled_points[0] + candidate_radius * sampled_normals[0]
        projection_matrix = np.eye(3) - np.outer(axis_unit_vector, axis_unit_vector)
        projected_center = np.matmul(projection_matrix, candidate_center)
        projected_points = np.matmul(projection_matrix, P.T).T
        euclidean_distances = np.linalg.norm(projected_points - projected_center, axis=1)
        inlier_mask = ((candidate_radius - inlier_threshold < euclidean_distances) &
                       (euclidean_distances < candidate_radius + inlier_threshold))
        inlier_count = np.count_nonzero(inlier_mask)
        if inlier_count > max_inliers_count:
            max_inliers_count = inlier_count
            best_cyl_center = projected_center
            best_cyl_axis = axis_unit_vector
            best_cyl_radius = candidate_radius
    print(f"Inliers:{max_inliers_count}")
    print(f"Center: {best_cyl_center}, Axis: {best_cyl_axis}, Radius: {best_cyl_radius}")

    return best_cyl_center, best_cyl_axis, best_cyl_radius
