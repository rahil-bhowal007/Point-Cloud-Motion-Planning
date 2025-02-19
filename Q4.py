from typing import Tuple
import numpy as np
import utils


def q4_a(M: np.ndarray, D: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Find transformation T such that D = T @ M. This assumes that M and D are
    corresponding (i.e. M[i] and D[i] correspond to same point)

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    use `np.linalg.svd` to perform singular value decomposition
    '''
    
    ### Enter code below
    source_mean = np.mean(M, axis=0)
    target_mean = np.mean(D, axis=0)

    centered_source = M - source_mean
    centered_target = D - target_mean

    covariance_matrix = np.dot(centered_source.T, centered_target)
    U, singular_values, Vt = np.linalg.svd(covariance_matrix)

    rotation_matrix = np.dot(Vt.T, U.T)
    translation_vector = target_mean.T - np.dot(rotation_matrix, source_mean.T)

    print("Rotation Matrix: ", rotation_matrix)
    print("Translation Vector: ", translation_vector)

    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation_vector

    return transformation_matrix


def q4_c(M: np.ndarray, D: np.ndarray) -> np.ndarray:
    '''
    Solves iterative closest point (ICP) to generate transformation T to best
    align the points clouds: D = T @ M

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    you should make use of the function `q4_a`
    '''

    ### Enter code below
    source_points_with_ones = np.concatenate([M, np.ones((M.shape[0], 1))], axis=1)
    target_points_with_ones = np.concatenate([D, np.ones((D.shape[0], 1))], axis=1)

    max_iterations = 50
    convergence_threshold = 0.001
    min_error = 0

    for _ in range(max_iterations):
        distances_between_points = np.linalg.norm(source_points_with_ones[:, np.newaxis, :] - target_points_with_ones, axis=-1)
        closest_target_indices = np.argmin(distances_between_points, axis=1)
        closest_target_points = target_points_with_ones[closest_target_indices]
        transformation_matrix = q4_a(source_points_with_ones[:, :3], closest_target_points[:, :3])
        source_points_with_ones = np.dot(source_points_with_ones, transformation_matrix.T)
        mean_error = np.mean(distances_between_points[np.arange(len(source_points_with_ones)), closest_target_indices])
        if np.abs(min_error - mean_error) < convergence_threshold:
            break
        min_error = mean_error

    final_transformation = q4_a(M, source_points_with_ones[:, :3])
    print(f"Final Transformation{final_transformation}")
    return final_transformation
