# LIO-SEGMOT

The official implementation of LIO-SEGMOT (**L**iDAR-**I**nertial **O**dometry
via **S**imultaneous **Eg**o-motion Estimation and **M**ultiple **O**bject
**T**racking). This project is based on the open source
[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) project.

![TEASER_GIF](./docs/_static/images/LIO-SEGMOT_KITTI_teaser.GIF)

You can check out [our video](https://youtu.be/5HtnDFPerVo) to understand the
main idea of LIO-SEGMOT.

> :rotating_light: ️The LIO-SEGMOT paper is submitted to [ICRA
> 2023](https://www.icra2023.org/) in Sept. 2022 and it is under review now. We
> do not publish this work yet, so please **DO NOT** share the code or any core
> concepts to other people that are not GPL or G-PAL members. Thank you!

- [:gear: Installation](#gear-installation)
  - [Step 1. Preparing the Dependencies](#step-1-preparing-the-dependencies)
  - [Step 2. Building the LIO-SEGMOT Project](#step-2-building-the-lio-segmot-project)
  - [Step 3. Preparing Object Detection Services](#step-3-preparing-object-detection-services)
- [:card_file_box: Sample Datasets](#card_file_box-sample-datasets)
- [:running_man: Run](#running_man-run)
- [:wheelchair: API of LIO-SEGMOT](#wheelchair-api-of-lio-segmot)
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
2. [gtsam 4.0.2](https://github.com/borglab/gtsam/tree/4.0.2)
   ```bash
   #!/bin/bash
   cd ~
   git clone -b 4.0.2 https://github.com/borglab/gtsam && cd gtsam
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

- [StephLin/SE-SSD-ROS (no license)](https://github.com/StephLin/SE-SSD-ROS)
  (based on [Vegeta2020/SE-SSD](https://github.com/Vegeta2020/SE-SSD))
- [StephLin/livox_detection_lio_segmot (GPL-3.0 license)](https://github.com/StephLin/livox_detection_lio_segmot)
  (based on [Livox-SDK/livox_detection](https://github.com/Livox-SDK/livox_detection))

Please refer to their installation instructions accordingly.

> We also provide an instrction to construct your own object detection service
> for LIO-SEGMOT, while the documentation plan is still working in progress.
> :smiling_face_with_tear:

## :card_file_box: Sample Datasets

We provide the following pre-built bags for KITTI raw sequences and the Hsinchu
dataset (GuangfuRoad sequence):

| Dataset | Sequence    | Link |
| ------- | ----------- | ---- |
| KITTI   | 0926-0009   | TBA  |
| KITTI   | 0926-0013   | TBA  |
| KITTI   | 0926-0014   | TBA  |
| KITTI   | 0926-0015   | TBA  |
| KITTI   | 0926-0032   | TBA  |
| KITTI   | 0926-0051   | TBA  |
| KITTI   | 0926-0101   | TBA  |
| Hsinchu | GuangfuRoad | TBA  |


## :running_man: Run

Please follow the steps to execute LIO-SEGMOT properly:

1. Launch the core LIO-SEGMOT service:
   ```bash
   #!/bin/bash
   # Please select one of the following configs to launch the service properly:
   # 1. With KITTI configuration
   roslaunch lio_sam run_kitti.launch

   # 2. With Hsinchu configuration
   roslaunch lio_sam run_hsinchu.launch

   # 3. Undefined (same as KITTI configuration)
   roslaunch lio_sam run.launch
   ```
2. Launch the selected object detection service:
   ```bash
   #!/bin/bash
   # SE-SSD-ROS & livox_detection_lio_segmot
   python3 ros_main.py
   ```
3. Start the customized ROS bag player:
   ```bash
   #!/bin/bash
   rosrun lio_sam lio_sam_offlineBagPlayer _bag_filename:="path/to/your/sequence.bag"
   ```

## :wheelchair: API of LIO-SEGMOT

> This section aims at telling you some APIs to access the results of
> LIO-SEGMOT, but I still need some time to write down them.
> :smiling_face_with_tear:

## :memo: Some Remarks

> This section aims at telling you some remarks of using LIO-SEGMOT, but I
> still need some time to write down them. :smiling_face_with_tear:
