import pycolmap
from pathlib import Path
import open3d as o3d
from PIL import Image
import numpy as np
import utils.utils as utils

# ---- settings ----
reconstruct = False     # timeintensive!
reconst_path = Path('data/reconstruction')
pcloud_path = Path('data/pclouds')
image_dir = Path('/media/ami162/ESD-USB/Exp1_smaller_lowflight')
#pcloud_name = "pcd_lowflight"
#pcloud_name = "pcd_complete"
pcloud_name = "pcd_pix4d"

# ---- programm ----
if reconstruct:
    utils.compute_pcloud(image_dir, reconst_path)   
pcloud = utils.convert_pcloud(reconst_path/"mvs"/"sparse", pcloud_path, pcloud_name)
pcloud, _ = utils.clean_pcloud(pcloud, flipz=True)
plants_bb, _ = utils.compute_plants(pcloud)

o3d.visualization.draw_geometries([pcloud]+ plants_bb, window_name=f'Pointcloud ({pcloud_name}) w/ Plant Bounding Boxes')
