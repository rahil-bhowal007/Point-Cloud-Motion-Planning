from typing import Tuple
import numpy as np

import utils


def q1_a(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a least squares plane by taking the Eigen values and vectors
    of the sample covariance matrix

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''

    ### Enter code below
    center = np.array([0,1,0])
    normal = np.array([0,0,1])
    center = np.mean(P, axis=0)
    print(P)
    print(center)
    print(f"Shape of P{P.shape}")
    transpose = P.T
    print(f"Shape of P.T{P.T.shape}")
    covariance_matrix = np.cov(transpose)
    print(covariance_matrix)
    eigen_values, eigen_vectors = np.linalg.eig(covariance_matrix)
    index = np.argmin(eigen_values)
    normal = eigen_vectors[:,index]
    return normal, center


def q1_c(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a plane using RANSAC

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    
    ### Enter code below
    center = np.array([0,1,0])
    normal = np.array([0,0,1])
    iterations = 2000
    epsilon = 0.01
    num_points = P.shape[0]
    best_score = 0
    best_normal = np.zeros(3)
    best_center = np.zeros(3)

    for _ in range(iterations):
        sample_indices = np.random.choice(num_points, 3, replace=False)
        sample_points = P[sample_indices]
        normal = np.cross(sample_points[1] - sample_points[0], sample_points[2] - sample_points[0])
        unit_normal_vector = normal / np.linalg.norm(normal)
        center = np.mean(sample_points, axis=0)
        projected_vector = np.dot(P - center, unit_normal_vector)
        euclidean_dists = np.abs(projected_vector)
        inliers = np.count_nonzero(euclidean_dists < epsilon)
        if inliers > best_score:
            best_normal = normal
            best_center = center
            best_score = inliers
    return best_normal, best_center
