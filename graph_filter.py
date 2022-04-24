import numpy as np 
import scipy.sparse
from collections import OrderedDict
import open3d as o3d

def sample_points(points, scores, n_samples, method):
    if method == 'prob':
        return sample_points_prob(points, scores, n_samples)
    elif method == 'topk':
        return sample_points_topk(points, scores, n_samples)

def sample_points_topk(points, scores, n_samples):
    """ sample points with top K scores.

    Args:
        points: np.ndarray 
            shape (N, 3)

        scores: np.ndarray
            score for each point . shape (N,)

        n_samples: int
            number of sampled points

    Returns:
        points_sampled: np.ndarray
            shape (n_samples, 3)
        
    """
    top_k = np.argsort(scores)
    ids_sampled = top_k[::-1][:n_samples]
    points_sampled = points[ids_sampled]

    return points_sampled

def sample_points_prob(points, scores, n_samples):
    """ sample points according to score normlized probablily

    Args:
        points: np.ndarray 
            shape (N, 3)

        scores: np.ndarray
            score for each point . shape (N,)

        n_samples: int
            number of sampled points

    Returns:
        points_sampled: np.ndarray
            shape (n_samples, 3)
        
    """
    scores = scores / np.sum(scores)
    ids_sampled = np.random.choice(
        points.shape[0], n_samples, replace=False, p=scores)
    points_sampled = points[ids_sampled]
    return points_sampled

def compute_resolution(kd_tree, pcd_np, scale_min_dist, scale_max_dist):
    """ compute radius for search
    Args:
        kdtree: o3d.geometry.KDTreeFlann
            kdtree for point cloud

        pcd_np: np.ndarray
            shape (N, 3)

        scale_min_dist: int/float
            scale the min distance to be searching radius

        scale_max_dist: int/float
            scale the max distance to be searching radius

    Returns:
        radius: float
            radius for searching neighbors
    """

    min_dist = np.inf
    max_dist = -np.inf
    N = pcd_np.shape[0]
    for i in range(N):
        [k, idx, dist_square] = kd_tree.search_knn_vector_3d(pcd_np[i], 2) # dist is squared distance
        min_dist = min(min_dist, dist_square[1])
        max_dist = max(max_dist, dist_square[1])
    min_dist = np.sqrt(min_dist)
    max_dist = np.sqrt(max_dist)
    
    radius = min(min_dist * scale_min_dist, max_dist * scale_max_dist)
    print(f"min_dist: {min_dist}, max_dist: {max_dist}, radius: {radius}")

    return radius

def compute_adjacency_matrix(kd_tree, pcd_np, radius, max_neighbor, var=None):
    """ compute adjacency matrix 
    Notice: for very sparse point cloud, you may not find any neighbor within radius,
    Then the row for this point in matrix is all zero. Will cause bug. 
    Maybe replace search_hybrid_vector_3d with search_knn_vector_3d
    

    Args:
        kdtree: o3d.geometry.KDTreeFlann(pcd)
            kdtree for point cloud

        pcd_np: np.ndarray
            shape (N, 3)

        radius: float
            searching radius for nearest neighbor

        max_neighbor: int
            max number of nearest neighbor

        var: float


    Returns:
        adj_matrix_new: scipy.sparse.coo_matrix

    """
    N = pcd_np.shape[0]
    adj_dict = OrderedDict()
    adj_matrix = scipy.sparse.dok_matrix((N, N))
    
    for i in range(N):
        [k, idx, dist_square] = kd_tree.search_hybrid_vector_3d(pcd_np[i], radius, max_neighbor)
        adj_matrix[i,idx] = dist_square
    
    adj_matrix = adj_matrix.tocoo()
    row, col, dist_square = adj_matrix.row, adj_matrix.col, adj_matrix.data

    dist = np.sqrt(dist_square)
    data_var = var if var is not None else np.var(dist)
    print(f"var: {data_var}")
    data_new = np.exp(- dist_square / data_var)

    adj_matrix_new = scipy.sparse.coo_matrix((data_new, (row, col)), shape=(N,N))

    return adj_matrix_new

def compute_D(W):
    """ compute degree matrix
    Args:
        W: scipy.sparse.coo_matrix

    Returns:
        D: sparse.matrix
    """
    N = W.shape[0]
    diag = np.array(W.sum(axis=1)).flatten()
    D = scipy.sparse.coo_matrix((N, N))
    D.setdiag(diag)

    return D

def apply_filter(pcd_np, F):
    """ 
    Args:
        pcd_np: np.ndarray 
            shape (N, 3)

        F: sparse.coo_matrix (N, N)
            graph filter

    Returns:
        scores: np.ndarray
            shape (N,)
    """
    scores = F @ pcd_np #  (N, 3).  X_i - sum( weight_ij * neighbor(X_i)_j )
    scores = np.linalg.norm(scores, ord=2, axis=1)  # [N], L2 distance
    scores = scores ** 2  # [N], L2 distance square 

    return scores



def compute_scores_from_points(pcd, filter_type, scale_min_dist, scale_max_dist, max_neighbor, var):
    """ compute scores for each point.

    """
    pcd_np = np.asarray(pcd.points)
    N = pcd_np.shape[0]
    kd_tree = o3d.geometry.KDTreeFlann(pcd)

    radius = compute_resolution(kd_tree, pcd_np, scale_min_dist, scale_max_dist)
    W = compute_adjacency_matrix(kd_tree, pcd_np, radius, max_neighbor, var)
    D = compute_D(W) 

    
    if filter_type == "all":
        F = scipy.sparse.coo_matrix((N, N))
        F.setdiag(1)

    elif filter_type == "high":
        I = scipy.sparse.coo_matrix((N, N))
        I.setdiag(1)
        row, col, data = D.row, D.col, D.data
        data_inv = 1 / data
        data_inv[np.isnan(data_inv)] = 0  # This step is not so reasonable. 
        D_inv = scipy.sparse.coo_matrix((data_inv, (row, col)), shape=(N, N))
        A = D_inv @ W  
        F = I - A

    else:
        raise("Not implemented")

    scores = apply_filter(pcd_np, F)
    return scores


def sample_pcd(pcd, filter_type, n_samples, scale_min_dist, scale_max_dist, max_neighbor, var, method = "prob"):
    """ sample_pcd
    Args:
        pcd: open3d.geometry.PointCloud

        var: float
            given variance. Set to None if use data's variance

    Returns:
        pcd_sampled: open3d.geometry.PointCloud
    """

    assert method in ['prob', 'topk']

    scores = compute_scores_from_points(pcd, filter_type, scale_min_dist, scale_max_dist, max_neighbor, var)

    pcd_np = np.asarray(pcd.points)
    points_sampled = sample_points(pcd_np, scores, n_samples, method)
    pcd_sampled = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points_sampled))

    return pcd_sampled
