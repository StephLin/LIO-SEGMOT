# LIO-SEGMOT

The official implementation of LIO-SEGMOT (**L**iDAR-**I**nertial **O**dometry
via **S**imultaneous **Eg**o-motion Estimation and **M**ultiple **O**bject
**T**racking). This project is based on the open source
[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) project.

![TEASER_GIF](./docs/_static/images/LIO-SEGMOT_KITTI_teaser.GIF)

You can check out [our video](https://youtu.be/5HtnDFPerVo) to understand the
main idea of LIO-SEGMOT.

This work is accepted for publication in [ICRA 2023](https://www.icra2023.org/).
If you use this project in your research, please cite:

```bibtex
@article{lin2023lio-segmot,
  title={Asynchronous State Estimation of Simultaneous Ego-motion Estimation and Multiple Object Tracking for LiDAR-Inertial Odometry},
  author={Lin, Yu-Kai and Lin, Wen-Chieh and Wang, Chieh-Chih},
  booktitle = {2023 International Conference on Robotics and Automation, {ICRA} 2023,
               London, UK, May 29 - June 2, 2023},
  pages     = {1--7},
  year      = {2023},
}
```

> We are preparing our paper now. We will release the preprint and update the
> citation later.

- [:gear: Installation](#gear-installation)
  - [Step 1. Preparing the Dependencies](#step-1-preparing-the-dependencies)
  - [Step 2. Building the LIO-SEGMOT Project](#step-2-building-the-lio-segmot-project)
  - [Step 3. Preparing Object Detection Services](#step-3-preparing-object-detection-services)
- [:card_file_box: Sample Datasets](#card_file_box-sample-datasets)
- [:running_man: Run](#running_man-run)
- [:wheelchair: Services of LIO-SEGMOT](#wheelchair-services-of-lio-segmot)
  - [`/lio_segmot/save_map`](#lio_segmotsave_map)
  - [`/lio_segmot/save_estimation_result`](#lio_segmotsave_estimation_result)
- [:memo: Some Remarks](#memo-some-remarks)

## :gear: Installation

The project is originally developed in **Ubuntu 18.04**, and the following
instruction supposes that you are using Ubuntu 18.04 as well. I am not sure if
it also works with other Ubuntu versions or other Linux distributions, but maybe
you can give it a try :+1:

Also, please feel free to open an issue if you encounter any problems of the
following instruction.

### Step 1. Preparing the Dependencies

Please prepare the following packages or libraries used in LIO-SEGMOT:

1. [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (w/ Desktop-Full
   Install) and the following dependencies:
   ```bash
   #!/bin/bash
   sudo apt update
   sudo apt install -y "ros-${ROS_DISTRO}-navigation" \
                       "ros-${ROS_DISTRO}-robot-localization" \
                       "ros-${ROS_DISTRO}-robot-state-publisher" \
                       "ros-${ROS_DISTRO}-jsk-recognition-msgs" \
                       "ros-${ROS_DISTRO}-jsk-rviz-plugins"
   ```
2. [gtsam 4.0.3](https://github.com/borglab/gtsam/tree/4.0.3)
   ```bash
   #!/bin/bash
   cd ~
   git clone -b 4.0.3 https://github.com/borglab/gtsam && cd gtsam
   mkdir build && cd build
   cmake ..
   sudo make install
   ```

### Step 2. Building the LIO-SEGMOT Project

You can use the following command to build the project:

```bash
#!/bin/bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:StephLin/LIO-SEGMOT.git
cd ..
catkin_make
```

### Step 3. Preparing Object Detection Services

We provide two object detection services for LIO-SEGMOT:

- [StephLin/SE-SSD-ROS (Apache-2.0 license)](https://github.com/StephLin/SE-SSD-ROS)
  (based on [Vegeta2020/SE-SSD](https://github.com/Vegeta2020/SE-SSD))
- [StephLin/livox_detection_lio_segmot (GPL-3.0 license)](https://github.com/StephLin/livox_detection_lio_segmot)
  (based on [Livox-SDK/livox_detection](https://github.com/Livox-SDK/livox_detection))

Please refer to their installation instructions accordingly.

> We also provide an instrction to construct your own object detection service
> for LIO-SEGMOT, while the documentation plan is still working in progress.
> :smiling_face_with_tear:

## :card_file_box: Sample Datasets

We provide the following pre-built bag files for KITTI raw sequences and the Hsinchu
dataset (GuangfuRoad sequence):

| Dataset | Sequence    | Bag File                                             | Ground Truth Trajectory                              |
| ------- | ----------- | ---------------------------------------------------- | ---------------------------------------------------- |
| KITTI   | 0926-0009   | [bag](http://140.113.150.180:5000/sharing/BrHtqyElq) | [tum](http://140.113.150.180:5000/sharing/HGebyDSCR) |
| KITTI   | 0926-0013   | [bag](http://140.113.150.180:5000/sharing/AQQa3kMSH) | [tum](http://140.113.150.180:5000/sharing/gREpr4xdI) |
| KITTI   | 0926-0014   | [bag](http://140.113.150.180:5000/sharing/HOgV5T79H) | [tum](http://140.113.150.180:5000/sharing/POuFRJBQI) |
| KITTI   | 0926-0015   | [bag](http://140.113.150.180:5000/sharing/XnoNLKSUQ) | [tum](http://140.113.150.180:5000/sharing/RBw1BeftU) |
| KITTI   | 0926-0032   | [bag](http://140.113.150.180:5000/sharing/ikbtkpWve) | [tum](http://140.113.150.180:5000/sharing/aQdaEnVjc) |
| KITTI   | 0926-0051   | [bag](http://140.113.150.180:5000/sharing/N1o9NcgU4) | [tum](http://140.113.150.180:5000/sharing/Wzu8QWoEC) |
| KITTI   | 0926-0101   | [bag](http://140.113.150.180:5000/sharing/lhhohwfJT) | [tum](http://140.113.150.180:5000/sharing/JCOaJHw04) |
| Hsinchu | GuangfuRoad | TBA                                                  | TBA                                                  |

Ground truth robot trajectories (based on GPS data provided by KITTI) are stored
as [the TUM format](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#ground-truth_trajectories).
Each row has 8 components containing timestamps (sec), xyz-position (meter), and
xyzw-orientation (quaternion):

```
timestamp x y z qx qy qz qw
```

## :running_man: Run

Please follow the steps to execute LIO-SEGMOT properly:

1. (Optional) Launch the ROS core:

   ```bash
   roscore
   ```

2. Launch the core LIO-SEGMOT service:

   ```bash
   #!/bin/bash
   # Please select one of the following configs to launch the service properly:
   # 1. With KITTI configuration
   roslaunch lio_segmot run_kitti.launch

   # 2. With Hsinchu configuration
   roslaunch lio_segmot run_hsinchu.launch

   # 3. Undefined (same as KITTI configuration)
   roslaunch lio_segmot run.launch
   ```

3. Launch the selected object detection service:
   ```bash
   #!/bin/bash
   # SE-SSD-ROS & livox_detection_lio_segmot
   python3 ros_main.py
   ```
4. Start the customized ROS bag player:
   ```bash
   #!/bin/bash
   rosrun lio_segmot lio_segmot_offlineBagPlayer _bag_filename:="path/to/your/sequence.bag"
   ```

## :wheelchair: Services of LIO-SEGMOT

### `/lio_segmot/save_map`

```txt
Usage: rosservice call /lio_segmot/save_map [RESOLUTION] [OUTPUT_DIR]

Example: rosservice call /lio_segmot/save_map 0.2 /path/to/a/directory/
```

This service saves LiDAR map to the local machine.

### `/lio_segmot/save_estimation_result`

```txt
Usage: rosservice call /lio_segmot/save_estimation_result
```

This service outputs current estimation results including

- `nav_msgs::Path robotTrajectory`: The robot trajectory
- (INTERNAL USE) `nav_msgs::Path[] objectTrajectories`: Trajectories for each object (indexed by the factor graph)
- (INTERNAL USE) `nav_msgs::Path[] objectVelocities`: Linear and angular velocities for each object (indexed by the factor graph)
- `nav_msgs::Path[] trackingObjectTrajectories`: Trajectories for each object (indexed by LIO-SEGMOT)
- `nav_msgs::Path[] trackingObjectVelocities`: Linear and angular velocities for each object (indexed by LIO-SEGMOT)
- `lio_segmot::ObjectStateArray[] trackingObjectStates`: States for each object during its lifetime (indexed by LIO-SEGMOT)
- (INTERNAL USE) `lio_segmot::flags[] objectFlags`: Flags for each object during its lifetime (indexed by the factor graph)
- `lio_segmot::flags[] trackingObjectFlags`: Flags for each object during its lifetime (indexed by LIO-SEGMOT)

in which custom types `lio_segmot::ObjectStateArray` (underlying `lio_segmot::ObjectState`) and `lio_segmot::flags` are given by

- `lio_segmot::ObjectStateArray`

  ```cpp
  Header header
  lio_segmot::ObjectState[] objects
  ```

- `lio_segmot::ObjectState`

  ```cpp
  Header header

  // The corresponding detection (measurement)
  jsk_recognition_msgs::BoundingBox detection

  // States of object pose and velocity in the factor graph
  geometry_msgs::Pose pose
  geometry_msgs::Pose velocity

  // Residual and innovation of the tightly-coupled detection factor
  bool hasTightlyCoupledDetectionError
  float64 tightlyCoupledDetectionError         // Residual
  float64 initialTightlyCoupledDetectionError  // Innovation

  // Residual and innovation of the loosely-coupled detection factor
  bool hasLooselyCoupledDetectionError
  float64 looselyCoupledDetectionError         // Residual
  float64 initialLooselyCoupledDetectionError  // Innovation

  // Residual and innovation of the smooth movement factor
  bool hasMotionError
  float64 motionError         // Residual
  float64 initialMotionError  // Innovation

  int32 index            // Object index
  int32 lostCount        // Counter of losing detections
  float64 confidence     // Detection's confidence score (given by detection methods)
  bool isTightlyCoupled  // Is the object tightly-coupled at this moment?
  bool isFirst           // Is the object just initialized at this moment?
  ```

- `lio_segmot/flags`

  ```cpp
  // Flags of the object in its lifetime
  int32[] flags  // 1: the object is tightly-coupled
                 // 0: the object is loosely-coupled
  ```

## :memo: Some Remarks

> This section aims at telling you some remarks of using LIO-SEGMOT, but I
> still need some time to write down them. :smiling_face_with_tear:
