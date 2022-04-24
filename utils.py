"""
    Adapted from https://github.com/pmkalshetti/fast_point_cloud_sampling/blob/master/utils.py
"""

import numpy as np
import open3d as o3d
import os
from collections import OrderedDict


def draw(geometries):
    vis = get_vis()
    if not isinstance(geometries, list):
        geometries = [geometries]
    for geometry in geometries:
        vis.add_geometry(geometry)
    vis.run()
    vis.destroy_window()


def read_data(path):
    pcd = o3d.io.read_point_cloud(path)
    print(f"Read {np.asarray(pcd.points).shape[0]} points")
    return pcd

def get_vis():
    vis = o3d.visualization.Visualizer()
    cwd = os.getcwd()  # to handle issue on OS X
    vis.create_window()
    os.chdir(cwd)  # to handle issue on OS X
    return vis

def write_pcd(pcd, path):
    dirname = os.path.dirname(path)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    o3d.io.write_point_cloud(path, pcd, write_ascii=True)



def create_box(n_points, size):
    mesh = o3d.geometry.TriangleMesh.create_box(width=size, height=size, depth=size)
    return mesh.sample_points_poisson_disk(n_points)

def create_cone(n_points, radius, height):
    mesh = o3d.geometry.TriangleMesh.create_cone(radius=radius, height=height)
    return mesh.sample_points_poisson_disk(n_points)

def create_cylinder(n_points, radius, height):
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
    return mesh.sample_points_poisson_disk(n_points)

def create_sphere(n_points, radius):
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    return mesh.sample_points_poisson_disk(n_points)