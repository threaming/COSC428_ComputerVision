import pycolmap
from pathlib import Path
import open3d as o3d
from PIL import Image
import numpy as np
import shutil

"""
Author: Andreas Ming

Utilities for 3D reconstruction and very simple, unrobust plant detection.

3D reconstruction based off COLMAPS
https://colmap.github.io/index.html
"""

# Function to remove points inside a bounding box
def remove_points_inside_box(pcd, bbox):
    """
    Remove points inside a bounding box (bbox) from a point cloud (pcd).

    Arguments:
    pcd         -- o3d point cloud
    bbox        -- o3d boudning box

    Returns:
    pcd_cleaned -- pointcloud w/o points in boudning box
    """
    points_np = np.asarray(pcd.points)
    mask = np.logical_or(np.any(points_np < bbox.min_bound, axis=1), np.any(points_np > bbox.max_bound, axis=1))
    pcd_cleaned = pcd.select_by_index(np.where(mask)[0])
    return pcd_cleaned

# create point bounding box
def create_bounding_box(centre_point, width, plane_parameters):
    """
    Create a bounding box around a point, according to a given width.
    The bounding box reaches from a given plane to the point.

    Arguments:
    centre_point     -- coordinate of centre point [X,Y,Z]
    width            -- width of bounding box (int)
    plane_parameters -- parameters of a fitted plane [a, b, c, d]

    Returns:
    bounding_box     -- o3d bounding box
    height           -- height of the bounding box
    """
    a, b, c, d = plane_parameters
    z = (-a*centre_point[0] - b*centre_point[1] - d) / c
    height = centre_point[2] - z
    min_bound = centre_point - np.array([width / 2, width / 2, 0])
    max_bound = centre_point + np.array([width / 2, width / 2, -height])
    return o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound), height

# compute furthest point from plane
def furthest_point_from_plane(pcd, plane_parameters):
    """
    Find the point furthest away from the fitted plane.

    Arguments:
    pcd              -- o3d point cloud
    plane_parameters -- parameters of a fitted plane [a, b, c, d]

    Returns:
    furthest_idx     -- index in pointcloud of the furthest point
    point coord      -- coordinates of furthest point
    """
    # Convert points to numpy array
    points_np = np.asarray(pcd.points)
    # Unpack plane parameters
    a, b, c, d = plane_parameters
    # Calculate signed distance from each point to the plane
    distances = (a * points_np[:, 0] + b * points_np[:, 1] + c * points_np[:, 2] + d) / np.sqrt(a**2 + b**2 + c**2)
    # Find index of furthest point
    furthest_idx = np.argmax(np.abs(distances))
    return furthest_idx, points_np[furthest_idx]

def clean_pcloud(pcd, flipz=False, nb_points=20, nb_radius=0.5):
    """
    Remove outliers and create bounding box, optionally flip z-axis.

    Arguments:
    pcd              -- o3d point cloud
    flipz            -- flip z-coordinate (default False)
    nb_points        -- neighbour points for outlier detection (default 20)
    nb_radius        -- neighbour radius for outlier detection (default 0.5)

    Returns:
    pcd              -- cleaned o3d point cloud
    bb               -- o3d bounding box
    """
    # --------------- remove outliers https://www.open3d.org/docs/latest/tutorial/Advanced/pointcloud_outlier_removal.html
    pcd, _ = pcd.remove_radius_outlier(nb_points=nb_points, radius=nb_radius)

    # --------------- flip z-axis
    if flipz:
        T = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])
        pcd.transform(T)
    bb = pcd.get_axis_aligned_bounding_box() # add bounding box
    bb.color = [255,0,0]
    #o3d.visualization.draw_geometries([pcloud, bb])

    return pcd, bb

# get bounding boxes of high objects
def compute_plants(pcd, hthresh=0.4):
    """
    From a given pointcloud very stupidly compute the plants
    through the highest points in the scene.

    Arguments:
    pcd              -- o3d point cloud
    hthresh          -- threshhold for minimum plant height

    Returns:
    plant_bb         -- list of detected bounding boxes
    plant_h          -- list of heights of detected plants
    """
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
    # --------------- create scene bounding box -------------
    bb = pcd.get_axis_aligned_bounding_box() # add bounding box
    bb.color = [1,0,0]
    
    #o3d.visualization.draw_geometries([pcloud, bb])

    # --------------- fit plane ----------------- (from labs)
    pcd_plane, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.5)
    # Fit a plane to the filtered volume using Open3D.
    plane_model, inliers = pcd_plane.segment_plane(distance_threshold=0.05,
                                            ransac_n=3,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Open3D plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    # From the fitted plane, find the points that lie inside and outside the plane.
    pcd_plane = pcd.select_by_index(inliers)
    pcd_plane.paint_uniform_color([1,0,0])  # Red
    
    # -------------- compute plant bounding boxes ----------
    height = 1
    pcdit = pcd
    plant_bb = []
    plant_h =[]
    while height >= hthresh:
        # ---------------- get furthest point --------------
        fidx, fcoord = furthest_point_from_plane(pcdit, plane_model)
        fpnt = pcdit.select_by_index([fidx])
        fpnt.paint_uniform_color([0,0,1])  # Blue
        # ---------------- create bounding box around point ----------
        fbb, height = create_bounding_box(fcoord, 0.7, plane_model)
        fbb.color = [1,0,0]
        plant_bb.append(fbb)
        plant_h.append(height)
        print(f"Found plant at: {fcoord}, {height} high")
        #o3d.visualization.draw_geometries([pcd, pcd_plane, fpnt, bb])

        pcdit = remove_points_inside_box(pcdit, fbb)

    return plant_bb, plant_h, pcd

# create pointcloud
def compute_pcloud(image_dir, output_path):
    """
    Use COLMAPS to compute a pointcloud from images.

    Arguments:
    image_dir    -- Path to images (max 2472x1648 w/o CUDA)
    output_path  -- Path to output directory (pointcloud results in there)
    """
    # Delete existing directory if it exists
    if output_path.exists():
        shutil.rmtree(output_path)
    output_path.mkdir(exist_ok=True)
    mvs_path = output_path / "mvs"
    database_path = output_path / "database.db"

    pycolmap.extract_features(database_path, image_dir)
    pycolmap.match_exhaustive(database_path)
    maps = pycolmap.incremental_mapping(database_path, image_dir, output_path)
    maps[0].write(output_path)

# convert pointcloud
def convert_pcloud(reconst_dir, output_path, pcd_name="points3D"):
    """
    Convert COLMAP pointcloud to PLY pointcloud for Open3D

    Arguments:
    reconst_dir  -- Directory of computed pointcloud
    output_path  -- Path to output directory
    pcd_name     -- Name of pointcloud

    Returns:
    pcloud       -- o3d pointcloud
    """
    reconstruction = pycolmap.Reconstruction(reconst_dir)
    print("---- Reconstruction Summary ----\n" +reconstruction.summary())
    pcd_name = pcd_name +'.ply'
    reconstruction.export_PLY(output_path/pcd_name)
    pcloud = o3d.io.read_point_cloud(str(output_path/pcd_name))
    return pcloud

# open pointcloud
def read_pcloud(pcd_path, pcd_name):
    pcloud = o3d.io.read_point_cloud(str(pcd_path/pcd_name)+".ply")
    return pcloud