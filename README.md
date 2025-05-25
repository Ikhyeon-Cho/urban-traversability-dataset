<div align="center">
<div>

# Urban Traversability Dataset

<br>

[🛠️ Download](#get-the-data) | [🎥 Video]() | [📖 Paper (RA-L)](https://ieeexplore.ieee.org/document/10468651) | [💻 How-to-train-model](https://github.com/Ikhyeon-Cho/LeSTA)

<br>

<div align="left">
<div>

The dataset corresponding to the paper *['Learning Self-supervised Traversability with Navigation Experiences of Mobile Robots: A Risk-aware Self-training Approach'](https://ieeexplore.ieee.org/document/10468651)* , accepted for publication in RA-L on Feb, 2024. 
<p align='center'>
    <img src="docs/Learned LiDAR Traversability.gif" alt="demo" width="800"/>
</p>

Our task of interests are: **terrain mapping**; **ground obstacle detection**; and the estimation of ***'robot-specific'* traversability**.

## Why we made custom dataset
While well-known datasets like [KITTI](https://www.cvlibs.net/datasets/kitti/) provide extensive data for robotic perception, they often fall short in addressing the specific needs of learning robot-specific traversability. That is, KITTI and similar datasets such as [Cityscapes](https://www.cityscapes-dataset.com/) and [nuScenes](https://www.nuscenes.org/), are mainly designed for general applications and may not capture the unique environmental and operational challenges faced by specific robots. 

**On the other hand, the robot's own navigation experiences provide rich contextual information which is crucial for navigating complex urban terrains.**

Therefore, we've made an automated data collection pipeline, which labels the traversed terrain maps by using a simple manual driving of the robot. All we have to do is just driving the robot in the target environment, which is typically done when building a prior map with existing SLAM systems. 

**Here are some good reasons of using onboard sensors and the robot's own navigation experiecne for learning robot-specific traversability:**
- **Data Scalability**: Leveraging the robot's own sensors and navigation experiences removes manual labeling efforts. Our approach enables continuous and automated data gathering, facilitating the creation of extensive datasets that capture diverse environmental conditions and scenarios. 
- **Robot-Environment Adaptability**: Data collected directly from the robot ensures that the training data is highly relevant to the specific robot and its operating environment. This allows the learned model to rapidly adapt to the target environment, with joint consideration of the robot's locomotion capabilities, leading to more accurate learning and predictions of traversability.

## About the dataset
<p align='center'>
    <img src="docs/robot.png" height="150"/>
    <img src="docs/label_generation.png" height="150" />
    <img src="docs/label_generation.gif" width="250" height="150"/>
</p>

- **Data Format:** Our datasets are provided as the files with [rosbag](https://wiki.ros.org/rosbag) format. For more information about the rosbag, see [rosbag/Tutorials](https://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data) and [rosbag/API Documentation](https://docs.ros.org/en/melodic/api/rosbag/html/).

- **What's in our dataset?:**   In each file of our datasets, the following ROS messages are provided:
  - LiDAR measurements (`velodyne_msgs/VelodyneScan`)
  - IMU measurements (`sensor_msgs::Imu`)
  - Odometry pose (`/tf`)
  - Extrinsic parameters of the sensors (`/tf_static`)
> **[Note]:** To reduce the size of datasets, only the *packet messages* of LiDAR sensor were recorded. This means that we have to unpack the lidar packets for playback the recorded point cloud measurements. For the purpose, there is a `vlp16packet_to_pointcloud.launch` file that handles the conversion of lidar packet to point clouds.
- **Robotic Platform:** Two-wheeled differential-drive robot, [ISR-M3](https://github.com/Ikhyeon-Cho/isr_robot_ros/tree/isr_m4/ros1), was used to collect the datasets. The robot was equipped with a single 3D LiDAR and IMU. During the experiments, 3D pose of the robot was estimated by the use of a Lidar-inertial odometry (LIO) system.

- **Environments:** We mainly provide two datasets with distinct ground surface characteristics. The training (blue) / testing (red) trajectories of a robot are shown in the aerial images below.   
  - **Urban campus**: This main target environment spans approximately 510m x 460m, with a maximum elevation change of 17m. The maximum inclination of the terrain is 14 degrees. The environment mostly consists of asphalt terrain. Some damaged roads, cobblestoned pavements, and the roads with small debris are challenging.
  - **Rural farm road**: We additionally validated our approach in the unstructured environments. Farm road areas were typically unpaved dirt or gravel and included various low-height ground obstacles. 

<p align='center'>
    <img src="docs/environments.jpeg" alt="demo" width="800"/>
</p>
<p align='center'>
    <img src="docs/parking_lot.gif" alt="demo" width="400"/>
    <img src="docs/farm_road.gif" alt="demo" width="400"/>
</p>



## Get the Data
### Download
Use the following links to download the datasets. 

**1. Urban Campus Dataset:** [[Google Drive](https://drive.google.com/drive/folders/183TBeVz4bP03MuXVHVSjRHlueewc1nlf?usp=drive_link)]
- [parking_lot.bag](https://drive.google.com/drive/folders/1nd85p5rsKTL74_4HLTLm7EenFjHPm12_?usp=drive_link) (-> 15 min)
- [multi_level_rampway.bag](https://drive.google.com/drive/folders/1CPQvUEKozZ9cKEjLJliN3kEovTb7-c3a?usp=drive_link) (-> 5 min)
- [campus_south.bag](https://drive.google.com/drive/folders/1V55vRC87sQaBAaWWjAhTvrjr_3OkEQxh?usp=drive_link) (-> 40 min)
- [campus_east.bag](https://drive.google.com/drive/folders/1V55vRC87sQaBAaWWjAhTvrjr_3OkEQxh?usp=drive_link) (-> 7 min)
- [campus_north.bag](https://drive.google.com/drive/folders/1V55vRC87sQaBAaWWjAhTvrjr_3OkEQxh?usp=drive_link)
- [campus_west.bag](https://drive.google.com/drive/folders/1V55vRC87sQaBAaWWjAhTvrjr_3OkEQxh?usp=drive_link)
- [campus_full.bag](https://drive.google.com/drive/folders/1LLqkmNG6afVPdyN_ACEZ2k6hXNu-UtkE?usp=drive_link) (-> about 50 min)
- [campus_full_long.bag](https://drive.google.com/drive/folders/1LLqkmNG6afVPdyN_ACEZ2k6hXNu-UtkE?usp=drive_link) (-> 2h 18 min)
- [campus_road_camera.bag](https://drive.google.com/drive/folders/1LLqkmNG6afVPdyN_ACEZ2k6hXNu-UtkE?usp=drive_link) (-> 10 min)
- [ku_innovation_hall_4F.bag](https://drive.google.com/drive/folders/1MuG2D3r42nBZszXDwN4JbErhAVHTSO8L?usp=drive_link) (-> 7 min)

**2. Farm Road Dataset:** [[Google Drive](https://drive.google.com/drive/folders/10agKQhQwF4uSaMfpA9IK7qBZYxF6g-aS?usp=drive_link)]
- [country_road_training.bag](https://drive.google.com/drive/folders/1YkoaqY1u1TsFm9jw4atJcMvaB4Ef20L-?usp=drive_link) (-> 4 min)
- [country_road_testing.bag](https://drive.google.com/drive/folders/1YkoaqY1u1TsFm9jw4atJcMvaB4Ef20L-?usp=drive_link) (-> 16 min)
- [greenhouse.bag](https://drive.google.com/drive/folders/1-oHQOaF4ARNNd7He0o5gFQpg_R4xsdIH?usp=drive_link) (-> 13 min)

### Play
We provide `terrain_dataset_player` ROS package for playing the datasets with the basic `rviz` visualization settings. Please follow the instructions below to play the recorded rosbag files.

### Dependencies
In order to run `terrain_dataset_player` package, please install the dependencies below:
- Ubuntu (tested on 20.04)
- [ROS](https://wiki.ros.org/noetic/Installation/Ubuntu) (tested on Noetic)
- `velodyne_pointcloud` (Velodyne ROS driver for unpacking LiDAR packet msgs)

Instaling the `velodyne_pointcloud` binaries by using `apt` should work through:
```
sudo apt install ros-noetic-velodyne-pointcloud
```



### Build
We recommend to use `catkin_tools` to build the ROS packages (It is not mandatory).
Instaling the `catkin_tools` package by using `apt` should work through:
```
sudo apt install python3-catkin-tools
```

Use the following commands to download and build the `terrain_dataset_player` package:
  ```
  cd ~/your-ros-workspace/src
  git clone https://github.com/Ikhyeon-Cho/urban-terrain-dataset.git
  cd ..
  catkin build terrain_dataset_player   ## If not using catkin_tools, use catkin_make
  ```


### Run the player
1. Locate the downloaded rosbag files (See [Download the datasets](#download)) into `data` folder.
2. Run the command below to play the rosbag files in `data` folder:
  ```
  ## Example: parking_lot.bag
  roslaunch terrain_dataset_player parking_lot.launch  playback_speed:=4.0
  ```

<!-- ## Acknowledgement -->


## Citation
Thank you for citing [our paper](https://ieeexplore.ieee.org/document/10468651) if this helps your research projects:
> Ikhyeon Cho, and Woojin Chung. **'Learning Self-Supervised Traversability with Navigation Experiences of Mobile Robots: A Risk-Aware Self-Training Approach'**, *IEEE Robotics and Automation Letters, 2024*.
```bibtex
@article{cho2024learning,
  title={Learning Self-Supervised Traversability With Navigation Experiences of Mobile Robots: A Risk-Aware Self-Training Approach}, 
  author={Cho, Ikhyeon and Chung, Woojin},
  journal={IEEE Robotics and Automation Letters}, 
  year={2024},
  volume={9},
  number={5},
  pages={4122-4129},
  doi={10.1109/LRA.2024.3376148}
}
```