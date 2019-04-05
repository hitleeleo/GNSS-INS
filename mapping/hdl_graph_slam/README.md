# hdl_graph_slam
Real-time 3D slam using a 3D LIDAR. It is based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection. It also utilizes floor plane detection to generate an environmental map with a completely flat floor. The Real time LiDAR odometry just work as a predictor for AMSI.

## Claim

hdl_graph_slam is adopted from Kenji Koide (koide@aisl.cs.tut.ac.jp). If there is any thing inappropriate, please feel freecontact me through 17902061r@connect.polyu.hk (Weisong WEN).

 Adaptive Multi Sensor Integration (ADMSI) for Autonomous Vehicles 
## Nodelets
***hdl_graph_slam*** consists of four nodelets. 
- *prefiltering_nodelet*
- *scan_matching_odometry_nodelet*
- *floor_detection_nodelet*
- *hdl_graph_slam_nodelet*

## save maps
rosservice call /hdl_graph_slam/save_map "resolution: 0.2
destination: '/home/wenws/mapfile2.pcd'"

## Cut bags
rosbag filter 2018-08-04-11-20-31.bag output.bag "t.secs >= 1533407042.19 and t.secs <= 1533407196.42"

The input point cloud is first downsampled by *prefiltering_nodelet*, and then passed to the next nodelets. While *scan_matching_odometry_nodelet* estimates the sensor pose by iteratively applying a scan matching between consecutive frames (i.e., odometry estimation), *floor_detection_nodelet* detects floor planes by RANSAC. The estimated odometry and the detected floor planes are sent to *hdl_graph_slam*. To compensate the accumulated error of the scan matching, it performs loop detection and optimizes a pose graph which takes odometry, loops, and floor planes into account.<br>

### Parameters
All the parameters are listed in *launch/hdl_graph_slam.launch* as ros params.

### Services
- */hdl_graph_slam/dump*  (std_srvs/Empty)
  - save all data (point clouds, floor coeffs, odoms, and pose graph) to the current directory.
- */hdl_graph_slam/save_map*  (hdl_graph_slam/SaveMap)
  - save generated map as a PCD file.

## Requirements
***hdl_graph_slam*** requires the following libraries:
- OpenMP
- PCL 1.7
- g2o

- g2o with ROS jade
Note that ***hdl_graph_slam*** cannot be built with older g2o libraries (such as ros-indigo-libg2o). ~~Install the latest g2o:~~
The latest g2o causes segfault. Use commit *a48ff8c42136f18fbe215b02bfeca48fa0c67507* instead of the latest one:

```bash
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout a48ff8c42136f18fbe215b02bfeca48fa0c67507
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RELEASE
make -j8
sudo make install
```

- g2o with ROS kinetic
 
```bash
1. sudo apt-get install ros-kinetic-libg2o 
2. sudo cp -r /opt/ros/kinetic/lib/libg2o_* /usr/local/lib 
3. sudo cp -r /opt/ros/kinetic/include/g2o /usr/local/include
```

 
The following ROS packages are required:
- geodesy
- nmea_msgs
- pcl_ros
- <a href="https://github.com/koide3/ndt_omp">ndt_omp</a>
```bash
sudo apt-get install ros-indigo-geodesy ros-indigo-pcl_ros ros-indigo-nmea-msgs


**[optional]** *bag_player.py* script requires ProgressBar2.
```bash
sudo pip install ProgressBar2
```


## Example2 (Outdoor)

Bag file (recorded in an outdoor environment): 
- [hdl_400.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz) (raw data, about 900MB)


```bash
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```

```bash
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
```

```bash
rosbag play --clock hdl_400.bag
```

