## What does this do?
This is a ROS package that generates RGB pointcloud given the pointcloud and the camera image. If a segmented camera image is provided, a marker indicating closest distance to the object is published

## Prerequisites
- Download the ROSbag of your choice. This should contain a pointcloud data and camera data with its info. Preferably the camera data is segmented.
- Provide the correct parameters for frames and topics in config/merge_pc.yaml

## Running
``roslaunch merge_cam_pc merge_pc.launch``
