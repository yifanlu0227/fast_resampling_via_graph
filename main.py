import open3d as o3d
from utils import read_data, draw, read_data
from graph_filter import sample_pcd
import numpy as np


if __name__ == "__main__":
    path_data = './data/box.ply'
    filter_type = "high" # or 'all'
    n_samples = 1000
    scale_min_dist = 5
    scale_max_dist = 5
    max_neighbor = 25
    var = 10 # float number or None
    method = 'topk' # 'topk' or 'prob'

    pcd_orig = read_data(path_data)
    pcd_orig.paint_uniform_color([0, 0, 0])

    pcd_sampled = sample_pcd(pcd_orig, 
                             filter_type, 
                             n_samples, 
                             scale_min_dist, 
                             scale_max_dist,
                             max_neighbor, 
                             var, 
                             method
                            )
    pcd_sampled.paint_uniform_color([1, 0, 0])

    draw([pcd_orig.translate([-25,0,0]), pcd_sampled])
