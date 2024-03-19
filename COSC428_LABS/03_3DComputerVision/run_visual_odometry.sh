#!/bin/bash

# Check if the orb_slam2 directory exists. If so, don't bother running the setup again.
if [ ! -d "visual_odometry" ]
then
    # Create a directory for storing all our files in.
    mkdir visual_odometry
    cd visual_odometry

    # Download the necessary files.
    while ! git clone https://eng-git.canterbury.ac.nz/sds53/cosc428-vo-example.git; do
        echo 'eng-git login failed, try again.' >&2;
    done

    # pip3 install wheel git+https://github.com/SamDSchofield/t3d_visualiser@main#egg=t3d_visualiser
    wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz

    # Extract the compressed files.
    tar -xf rgbd_dataset_freiburg1_desk.tgz

    # Run the association script on the colour and depth images from the dataset and store the output in associations.txt.
    python ./cosc428-vo-example/associate.py rgbd_dataset_freiburg1_desk/rgb.txt rgbd_dataset_freiburg1_desk/depth.txt > rgbd_dataset_freiburg1_desk/associations.txt
else
    # If the setup has already been run, then just move to the visual_odometry directory in preparation for running the visual odometry algorithm.
    cd visual_odometry
fi

# Run the ORB_SLAM2 algorithm on the dataset we just downloaded.

python3 ./cosc428-vo-example/vo.py rgbd_dataset_freiburg1_desk ./results.txt ./cosc428-vo-example/cfg/tum/fr1.yaml

# Once we're done, return to the original directory that we started from.
cd ..

