# d435_parameters.py

# Make Python look for the right place for pyrealsense2 for the COSC lab PCs.
import sys
sys.path.append('/usr/local/lib') 

# First import the library
import pyrealsense2 as rs
import time
import json
import numpy as np
import cv2

def load_camera_parameters(advanced_mode_object, json_file_path):
    with open(json_file_path, 'r') as json_out:
        parameters = json.load(json_out)
    # The C++ JSON parser requires double-quotes for the json object so we need
    # to replace the single quote of the pythonic json to double-quotes
    json_string = str(parameters).replace("'", '\"')
    advanced_mode_object.load_json(json_string)

# Configure video streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
print("Depth Scale is: {:.4f}m".format(depth_scale))

# To start with, the camera must be in "Advanced Mode".
# If this is not enabled, enable it first.
dev = rs.context().query_devices()[0]  # Assuming that there's only one camera plugged in.
advanced_mode_object = rs.rs400_advanced_mode(dev)
print("Advanced mode is", "enabled" if advanced_mode_object.is_enabled() else "disabled")

# Loop until we successfully enable advanced mode
while not advanced_mode_object.is_enabled():
    advanced_mode_object.toggle_advanced_mode(True)
    # The camera resets here, so we wait a bit before trying to connect again.
    print("Sleeping for 5 seconds...")
    time.sleep(5)
    dev = rs.context().query_devices()[0]
    advanced_mode_object = rs.rs400_advanced_mode(dev)

# Camera parameter presets for comparison
presets = ['DefaultPreset_D435.json',
           'EdgeMapD435.json', 
           'HandGesturePreset.json', 
           'HighAccuracyPreset.json', 
           'HighDensityPreset.json', 
           'MidDensityPreset.json', 
           'ShortRangePreset.json']

i = 0
frame_counter = 0
while cv2.waitKey(1) < 0:

    #Load a new parameter preset every 150 frames until 'q' is pressed
    if frame_counter % 150 == 0:
        print('Loading {}'.format(presets[i])) 
        load_camera_parameters(advanced_mode_object, 'd435_presets/' + presets[i])
        i = (i + 1) % len(presets)
        
    # Wait for a coherent set of frames.
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth/infrared images for visibilty (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # Stack images horizontally
    images = np.hstack((color_image, depth_colormap))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    frame_counter += 1

