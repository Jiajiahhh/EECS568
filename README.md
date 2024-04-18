# ROB530Final

## Installation
### Semantic Segmentation
This model (AR-Net) is an extension of Segformer, please refer to https://github.com/NVlabs/SegFormer/tree/master for the Segforomer backbone.

For a detailed implmentation of AR-Net, refer to https://github.com/MapleXE/AR-Net.

There are two environments that can be used for elevation mapping: rosbag, and simulation.

In the coordinate_map folder, the semanticPub.py is used to subscribe the rgb camera and perform segmentation. You may need to change the input_topic (e.g. if you want to test on simulation, uncomment all lines with ##for simulation, and comment out all lines with ##for rosbag), model path & config file, and the device(default:cuda).

To build the coordinate_map.cpp you need to first download pcl (https://github.com/PointCloudLibrary/pcl), then follow their instructions to build the package locally and modify the CMakeLists' PCL_DIR correctly. 

A semantic point cloud is generated in coordinate_map.cpp for elevation mapping. You may need to modify the code to switch between rosbag test or simulation test.

### Elevation Map
Mapping is based on prior work from ETH, follow their instructions to setup the environment requirement for our code. https://github.com/ANYbotics/elevation_mapping

## Test
Use the following commands to run the test demo.
```
roslaunch elevation_mapping_demos turtlesim3_waffle_demo.launch #simulation
roslaunch elevation_mapping_demos scout_base_t265.launch #field test rosbag
python3 semanticPub.py #This may need additional environment requirement
```
Use the following command to run the visual odometry in simulation.
```
rosrun visodom visodo_ros_v2
```
Use the following command to run the visual odometry in real environment.
```
rosrun visodom visodo_ros_real
```
### Data & Model Config
Data for rosbag can be downloaded here: https://drive.google.com/file/d/1AeSp0Hy8wSHd3G4FlrhOfOEF4HTDPf89/view?usp=drive_link
The trained segmentation model can be found here: https://drive.google.com/file/d/1gBVYd2bt254pQJ-HnQFMDsQR2o6G2Xhf/view?usp=drive_link
The model configuration file can be found here: https://drive.google.com/file/d/1VzgHvvhtghwegc9KhhZg6f8mr4MAyMR9/view?usp=drive_link

## IMU Correction InEKF
The data in this part is obtained from visual odometry and saved in IMU_InEKF/data.
We provide two sets of test data. 'Output.txt' stores the VO processed data (position (x, y), yaw, dt, feature point coordinates), and 'true.txt' stores the (true position of the car (x, y). imu readings: (quaternion, angular velocity, linear acceleration))

### Test
Replace the data by changing the addresses of 'filetrue' and 'filepath2' in run_imu_riekf.py.
Use the following commands to run the test demo.
```
python run_imu_riekf.py
```

