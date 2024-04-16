# ROB530Final

## Installation
### Semantic Segmentation
This part is an extension of Segformer, please refer to https://github.com/NVlabs/SegFormer/tree/master for backbone.
Please refer to https://github.com/MapleXE/AR-Net for deatiled implementation.

In the coordinate_map package, the semanticPub.py is used to subscribe the rgb camera and do segmentation. You may need to change the topic(if you want to test on simulation), model path & config file, and the device(default:cuda)

To build the coordinate_map.cpp you need to first download and build pcl (https://github.com/PointCloudLibrary/pcl) local and modify the CMakeLists' PCL_DIR correctly. 

In coordinate_map.cpp we will generate semantic pointcloud for mapping. You may need to modify the code to switch from rosbag test or simulation test.
### Elevation Map
This part is based on ETH work. Please use our code and follow their instructions to setup the environment requirement. https://github.com/ANYbotics/elevation_mapping

## Test
```
roslaunch elevation_mapping_demos turtlesim3_waffle_demo.launch #simulation
roslaunch elevation_mapping_demos turtlesim3_waffle_demo.launch #field test rosbag
python3 semanticPub.py #This may need additional environment requirement
```
The rosbag sample can be downloaded here: https://drive.google.com/file/d/1AeSp0Hy8wSHd3G4FlrhOfOEF4HTDPf89/view?usp=drive_link
The trained segmentation model and configure file: https://drive.google.com/file/d/1gBVYd2bt254pQJ-HnQFMDsQR2o6G2Xhf/view?usp=drive_link; https://drive.google.com/file/d/1VzgHvvhtghwegc9KhhZg6f8mr4MAyMR9/view?usp=drive_link

