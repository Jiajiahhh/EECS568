# ROB530Final

## Installation
### Semantic Segmentation
This part is an extension of Segformer, please refer to https://github.com/NVlabs/SegFormer/tree/master for backbone.
Please refer to https://github.com/MapleXE/AR-Net for deatiled implementation.

In the coordinate_map package, the semanticPub is used to subscribe the rgb camera and do segmentation. You may need to change the topic(if you want to test on rosbag), model path & config file, and the device(default:cpu)

To build the coordinate_map.cpp you need to first download and build pcl (https://github.com/PointCloudLibrary/pcl) local and modify the CMakeLists' PCL_DIR correctly. 
In coordinate_map we will generate semantic pointcloud for mapping. You may need to modify subscribe topic names.
### Elevation Map
This part is based on ETH work. Please use our code and follow their instructions to setup the environment requirement.
https://github.com/ANYbotics/elevation_mapping
