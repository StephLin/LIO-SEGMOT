# LIO-SEGMOT

The official implementation of LIO-SEGMOT (**L**iDAR-**I**nertial **O**dometry
via **S**imultaneous **Eg**o-motion Estimation and **M**ultiple **O**bject
**T**racking), an optimization-based odometry approach targeted for dynamic
environments. LIO-SEGMOT can provide continuous object tracking results while
preserving the keyframe selection mechanism in the odometry system. This work is
accepted for publication in [ICRA 2023](https://www.icra2023.org/).

![TEASER_GIF](./docs/_static/images/LIO-SEGMOT_KITTI_teaser.GIF)

You can check out [our video](https://youtu.be/5HtnDFPerVo) to understand the
main idea of LIO-SEGMOT. For more, please refer to our paper:

- Yu-Kai Lin, Wen-Chieh Lin, Chieh-Chih Wang, **Asynchronous State Estimation of Simultaneous Ego-motion Estimation and Multiple Object Tracking for LiDAR-Inertial Odometry**. _2023 International Conference on Robotics and Automation (ICRA)_, pp. 10616--10622, May 2023. ([paper](https://doi.org/10.1109/ICRA48891.2023.10161269)) ([preprint](https://gpl.cs.nctu.edu.tw/Steve-Lin/LIO-SEGMOT/preprint.pdf)) ([code](https://github.com/StephLin/LIO-SEGMOT)) ([video](https://youtu.be/5HtnDFPerVo))

If you use this project in your research, please cite:

```bibtex
@article{lin2023lio-segmot,
  title={Asynchronous State Estimation of Simultaneous Ego-motion Estimation and Multiple Object Tracking for LiDAR-Inertial Odometry},
  author={Lin, Yu-Kai and Lin, Wen-Chieh and Wang, Chieh-Chih},
  booktitle = {2023 International Conference on Robotics and Automation, {ICRA} 2023,
               London, UK, May 2023},
  pages     = {10616--10622},
  year      = {2023},
}
```

- [:newspaper: Poster](#newspaper-poster)
- [:gear: Installation](#gear-installation)
  - [Step 1. Preparing the Dependencies](#step-1-preparing-the-dependencies)
  - [Step 2. Building the LIO-SEGMOT Project](#step-2-building-the-lio-segmot-project)
  - [Step 3. Preparing Object Detection Services](#step-3-preparing-object-detection-services)
- [:card_file_box: Sample Datasets](#card_file_box-sample-datasets)
- [:running_man: Run](#running_man-run)
- [:wheelchair: Services of LIO-SEGMOT](#wheelchair-services-of-lio-segmot)
  - [`/lio_segmot/save_map`](#lio_segmotsave_map)
  - [`/lio_segmot/save_estimation_result`](#lio_segmotsave_estimation_result)
- [:memo: Remarks](#memo-remarks)
  - [Hyperparameters](#hyperparameters)
    - [Hierarchical Criterion](#hierarchical-criterion)
    - [Factor Graph Optimization](#factor-graph-optimization)
    - [High-speed Moving Object Supports in Early Steps](#high-speed-moving-object-supports-in-early-steps)
    - [Lifecycle Management of Tracking Objects](#lifecycle-management-of-tracking-objects)
  - [Limitations of LIO-SEGMOT](#limitations-of-lio-segmot)
  - [Possible Future Research Directions](#possible-future-research-directions)
- [:gift: Acknowledgement](#gift-acknowledgement)

## :newspaper: Poster

![Poster](./docs/_static/images/poster.png)

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
| Hsinchu | GuangfuRoad | [bag](http://gofile.me/56KxB/mz6HOYOMG)              | [tum](http://gofile.me/56KxB/t5smaOAZk)              |

If you cannot download sample datasets from above links, please refer to
[this alternative link (Google Drive)](https://drive.google.com/drive/folders/17QwfTMCv-2XdgOLxGdrnCwybcSYQhs7S?usp=drive_link).

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
   # Please check their documentation to see how they are launched
   ```

4. Start the customized ROS bag player:

   ```bash
   #!/bin/bash
   rosrun lio_segmot lio_segmot_offlineBagPlayer _bag_filename:="path/to/your/sequence.bag"
   ```

   The default registered LiDAR and IMU topics are `/points_raw` and `/imu_raw`,
   respectively. If you want to register other LiDAR/IMU topics, please add
   additional options `_lidar_topic` and `_imu_topic`. For example, if you are
   using the GuangfuRoad sequence (`/velodyne_points` and `/imu/data` for LiDAR
   and IMU topics, respectively):

   ```bash
   rosrun lio_segmot lio_segmot_offlineBagPlayer _bag_filename:="GuangfuRoad-06-13.bag" \
                                                 _lidar_topic:="/velodyne_points" \
                                                 _imu_topic:="/imu/data"
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
- $\color{gray}\textsf{(INTERNAL USE)}$ `nav_msgs::Path[] objectTrajectories`: Trajectories for each object (indexed by the factor graph)
- $\color{gray}\textsf{(INTERNAL USE)}$ `nav_msgs::Path[] objectVelocities`: Linear and angular velocities for each object (indexed by the factor graph)
- `nav_msgs::Path[] trackingObjectTrajectories`: Trajectories for each object (indexed by LIO-SEGMOT)
- `nav_msgs::Path[] trackingObjectVelocities`: Linear and angular velocities for each object (indexed by LIO-SEGMOT)
- `lio_segmot::ObjectStateArray[] trackingObjectStates`: States for each object during its lifetime (indexed by LIO-SEGMOT)
- $\color{gray}\textsf{(INTERNAL USE)}$ `lio_segmot::flags[] objectFlags`: Flags for each object during its lifetime (indexed by the factor graph)
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

## :memo: Remarks

### Hyperparameters

There are various covariance matrices designed for the hirarchical criterion
(innovation filtering) and factor graph optimization (increments of LIO-SEGMOT
w.r.t. LIO-SAM) in LIO-SEGMOT. This section is going to explain them.

> All covariance matrices in settings are expressed as diagonal vectors.

Taking [the KITTI configuration](./config/params_kitti.yaml) for example, we have the following settings:

#### Hierarchical Criterion

This section collects settings for the hierarchical criterion. It can be viewed
as a kind of innovation filtering. In brief, the criterion is designed to
progressively make the following decisions when a new detection
$\boldsymbol{z}\in SE(3)$ is coming into the system:

| ID       | Description                                                                 |
| -------- | --------------------------------------------------------------------------- |
| **(Q1)** | Does the detection belong to any existing object $\boldsymbol{x}_{t,i}$?    |
| **(Q2)** | If Q1 holds, does $\boldsymbol{z}$ follows the $i$-th object's motion?      |
| **(Q3)** | If Q1 and Q2 holds, should the tightly-coupled detection factor be applied? |

The first two questions **(Q1)** and **(Q2)** are determined by using the
Mahalanobis distance of the error vector,

$$
\Big\Vert\text{ detection error of }\boldsymbol{z}\text{ and the }i\text{-th object } \boldsymbol{x} _{t,i}\text{ }\Big\Vert _{\Sigma}
\leq \varepsilon,
$$

with given covariance with given covariance matrices
$\Sigma\in\{\Sigma_ {\text{Q}_ 1},\Sigma_ {\text{Q}_ 2}\}\subsetneq\mathbb{R}^{6\times 6}$
and a threshold $\varepsilon>0$. We assume that
$\Sigma_ {\text{Q}_ 2}-\Sigma_ {\text{Q}_ 1}$
is positive semidefinite (PSD), i.e.,
$\Sigma_ {\text{Q}_ 2}-\Sigma_{\text{Q}_ 1} \succeq 0$,
to prevent ambiguity of the hierarchical criterion that **(Q2)** holds but
**(Q1)** does not hold.

Two spatial information-based tests are conducted to determine **(Q3)**, which
are the detection constraint and the velocity constraint:

- **(Detection Constraint)** The above equation holds with another given
  covariance matrix
  $\Sigma_ {\text{Q}_ {3,1}}$
  that satisfies
  $\Sigma_ {\text{Q}_ {3,1}}-\Sigma_ {\text{Q}_ {2}} \succeq 0$.

- **(Velocity constraint)** The variance of velocities in previous steps is
  small enough. That is,

  $$\frac{1}{N}\sum_ {s=1}^{N} \Big\Vert \text{Log}(\boldsymbol{v}_ {t-s,i}) - \text{Log}(\bar{\boldsymbol{v}}_ {t,i}) \Big\Vert_ {\Sigma_{Q_ {3,2}}}^2 \leq \varepsilon$$

  with a given covariance matrix $\Sigma_{Q_{3,2}}$, where $N$ is the fixed
  number of previous velocities of object states and
  $\bar{\boldsymbol{v}}_{t,i}\in SE(3)$ is the mean of the $N$ previous
  velocities.

If **(Q1)** holds for the detection $\boldsymbol{z}$ and the corresponding
$i$-th object, the new state of the $i$-th object along with a loosely-coupled
detection factor would be added to the factor graph.

Furthermore, if **(Q2)**
holds, a constant velocity factor and a smooth movement factor would be also
added to the factor graph.

Finally, if **(Q3)** holds, the loosely-coupled
detection factor would be replaced with a tightly-coupled detection factor. It
means that the $i$-th object are regarded as a reliable object that are suitable
to refine the odometry.

| Notation                           | Setting                                             | Description                                                                                                                                                                                                                                   | Default Value                                      |
| ---------------------------------- | --------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| $\varepsilon$                      | `detectionMatchThreshold`                           | The threshold to classify all Mahalanobis distances in the hirarchical criterion (except for the tightly-coupled detection factor).                                                                                                           | 19.5                                               |
| $\Sigma_{\text{Q}_1}$              | `dataAssociationVarianceVector`                     | The covariance matrix to determine if a detection belongs to a given object. This covariance is used to maintain tracking ID.                                                                                                                 | `[3.0e-4, 3.0e-4, 3.0e-4, 5.0e-2, 3.0e-2, 3.0e-2]` |
| $\Sigma_{\text{Q}_2}$              | `looselyCoupledMatchingVarianceVector`              | The covariance matrix to determine if a detection follows the object's motion.                                                                                                                                                                | `[1.0e-4, 1.0e-4, 1.0e-4, 2.0e-3, 2.0e-3, 2.0e-3]` |
| $\varepsilon^\prime$               | `tightCouplingDetectionErrorThreshold`              | The threshold to classify the Mahalanobis distance in the detection constraint of the tightly-coupled detection factor checks.                                                                                                                | 26.0                                               |
| $\Sigma_{\text{Q}_{3,1}}^\prime$   | `tightlyCoupledMatchingVarianceVector`              | The covariance to determine if a detection satisfies the detection constraint in the tightly-coupled detection factor checks.                                                                                                                 | `[8.0e-6, 8.0e-6, 8.0e-6, 1.0e-4, 1.0e-4, 1.0e-4]` |
| $N$                                | `numberOfVelocityConsistencySteps`                  | The number of samples used in velocity constraint of the tightly-coupled detection checks.                                                                                                                                                    | 4                                                  |
| $N^\prime$                         | `numberOfPreLooseCouplingSteps`                     | The number of steps that objects should only use loosely-coupled detection factors (due to velocity constraint of the tightly-coupled detection checks, see [below](#high-speed-moving-object-supports-in-early-steps) for more information). | 6                                                  |
| $\sigma_{\text{Q}_{3,2}}^\text{A}$ | `objectAngularVelocityConsistencyVarianceThreshold` | The angular part of the covariance matrix to determine if the object satisfies the velocity constraint in the tightly-coupled detection factor checks.                                                                                        | 1.0e-5                                             |
| $\sigma_{\text{Q}_{3,2}}^\text{L}$ | `objectLinearVelocityConsistencyVarianceThreshold`  | The linear part of the covariance matrix to determine if the object satisfies the velocity constraint in the tightly-coupled detection factor checks.                                                                                         | 1.0e-2                                             |

For engineering purposes, we use two thresholds $\varepsilon$ and
$\varepsilon^\prime$ in the implementation. In addition, we decouple the angular
part and the linear part of $\Sigma_{\text{Q}_{3,2}}$. The following equations
are shown to coincide with the expression used in our paper:

$$
\begin{aligned}
\displaystyle\Sigma _{\text{Q} _{3,1}} &= \left(\frac{\varepsilon^\prime}{\varepsilon}\right)^2 \cdot \displaystyle\Sigma _{\text{Q} _{3,1}}^\prime, \\
\displaystyle\Sigma _{\text{Q} _{3,2}}^\prime &= \begin{bmatrix}\sigma _{\text{Q} _{3,2}}^\text{A} \\
& \sigma _{\text{Q} _{3,2}}^\text{A} \\
&& \sigma _{\text{Q} _{3,2}}^\text{A} \\
&&& \sigma _{\text{Q} _{3,2}}^\text{L} \\
&&&& \sigma _{\text{Q} _{3,2}}^\text{L} \\
&&&&& \sigma _{\text{Q} _{3,2}}^\text{L}
\end{bmatrix}, \\
\displaystyle\Sigma _{\text{Q} _{3,2}} &= \frac{1}{\varepsilon^2} \cdot \displaystyle\Sigma _{\text{Q} _{3,2}}^\prime.
\end{aligned}
$$

#### Factor Graph Optimization

Covariance matrices used in factor graph optimization. Different from the above
section, they are essential to "balance" different types of measurements in a
unified factor graph. Those matrices are relatively rare to be modified.

| Notation             | Setting                                  | Description                                                                                   | Default Value                                      |
| -------------------- | ---------------------------------------- | --------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| $\Sigma_{\text{C}}$  | `constantVelocityDiagonalVarianceVector` | The covariance matrix of constant velocity factors used in factor graph optimization.         | `[2.0e-4, 2.0e-4, 1.0e-3, 2.0e-1, 1.0e-1, 1.0e-1]` |
| $\Sigma_{\text{M}}$  | `motionDiagonalVarianceVector`           | The covariance matrix of smooth movement factors used in factor graph optimization.           | `[2.0e-4, 2.0e-4, 1.0e-3, 1.0e-1, 1.0e-2, 1.0e-2]` |
| $\Sigma_{\text{LC}}$ | `looselyCoupledDetectionVarianceVector`  | The covariance matrix of loosely-coupled detection factors used in factor graph optimization. | `[2.0e-4, 2.0e-4, 2.0e-4, 1.5e-3, 1.5e-3, 1.5e-3]` |
| $\Sigma_{\text{TC}}$ | `tightlyCoupledDetectionVarianceVector`  | The covariance matrix of tightly-coupled detection factors used in factor graph optimization. | `[2.0e-4, 2.0e-4, 2.0e-4, 1.5e-3, 1.5e-3, 1.5e-3]` |

#### High-speed Moving Object Supports in Early Steps

As all tracking objects' velocities are initialized with zero-speed, it may be
hard to associate detections in different moments for high-speed moving objects
in the early stage. To mitigate this issue, a larger covariance matrix for
**(Q2)** in the hierarchical criterion is used in the first few steps to
accommodate objects that are moving fast.

Since those steps are used to figure out the inital speed of an object, we do
not account them for the velocity constraint in the tightly-coupled detection
factor checks. Therefore, we have $N^\prime = N + N^\text{E}$.

| Notation                       | Setting                                       | Description                                                                                   | Default Value                                      |
| ------------------------------ | --------------------------------------------- | --------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| $N^\text{E}$                   | `numberOfEarlySteps`                          | Number of the first steps to accommodate high-speed moving objects.                           | 2                                                  |
| $\Sigma_{\text{Q}_2}^\text{E}$ | `earlyLooselyCoupledMatchingVarianceVector`   | The covariance matrix to determine if a detection follows the object's motion.                | `[3.0e-4, 3.0e-4, 3.0e-4, 5.0e-2, 5.0e-3, 5.0e-3]` |
| $\Sigma_{\text{C}}^\text{E}$   | `earlyConstantVelocityDiagonalVarianceVector` | The covariance matrix of loosely-coupled detection factors used in factor graph optimization. | `[2.0e-4, 2.0e-4, 1.0e-3, 2.0e-1, 1.0e-1, 1.0e-1]` |

#### Lifecycle Management of Tracking Objects

In real world applications, it's likely to lose detections of tracking objects
due to occlusion or other complicate environment circumstances. To mitigate
frequent ID-switching in multiple object tracking due to the above issue, we
still track each object for a little while, even though they do not have any
corresponding detections.

| Notation     | Setting                      | Description                                                                                | Default Value |
| ------------ | ---------------------------- | ------------------------------------------------------------------------------------------ | ------------- |
| $N^\text{L}$ | `trackingStepsForLostObject` | Number of steps that LIO-SEGMOT still keeps an object of missing detections in the system. | 3             |

### Limitations of LIO-SEGMOT

Currently, most state-of-the-art object detection approaches (e.g.,
[SE-SSD](https://arxiv.org/abs/2104.09804),
[PointPillars](https://arxiv.org/abs/1812.05784),
[PointRCNN](https://arxiv.org/abs/1812.04244),
[PV-RCNN](https://arxiv.org/abs/1912.13192),
[SPG](https://arxiv.org/abs/2108.06709), and
[ST3D](https://arxiv.org/abs/2103.05346)) are constructed under machine
learning-based neural network architectures, while the issue of domain
adaptation for data-driven object detection approaches still remains a
challenging open problem targeted by recent researches (e.g,
[SPG](https://arxiv.org/abs/2108.06709) and
[ST3D](https://arxiv.org/abs/2103.05346)). That is, the performance of object
detection models change varying under different geographic appearances or
weather conditions.

[The model weight of SE-SSD](https://github.com/Vegeta2020/SE-SSD) used in
LIO-SEGMOT is trained under a subset of the KITTI dataset, and thus it might
work well in other subsets of the KITTI dataset. However, we can observe that
there are numerous false positive detections in the Hsinchu dataset. Therefore,
it forces us to choose different detection models (i.e.,
[PointPillars w/ Livox's model weights](https://github.com/Livox-SDK/livox_detection))
when experimenting LIO-SEGMOT in different real world datasets. In addition,
since the model still cannot perform as good detection results in the Hsinchu
dataset as SE-SSD does in the KITTI dataset, we need to use a more strict
criterion for the detection constraint in the tightly-coupled detection factor
checks (by decreasing $\varepsilon^\prime$ from 26.0 to 19.0). This points out
the first limitation of LIO-SEGMOT. That is, covariance matrices and thresholds
related to object detections (mainly hyperparameters in the hierarchical
criterion) are required to be adjusted according to the stability of object
detections. Despite it affects generalization capability of the proposed method,
we believe that the problem can be mitigated with the breakthrough of the domain
adaptation for 3-D object detection.

The second limitation of LIO-SEGMOT is related to the motion model of tracking
objects. If an object does not move at constant velocity, LIO-SEGMOT may
miscalculate the object velocity, leading to inaccurate predicted object pose in
the subsequent state. The main reason is that objects' velocities are supposed
to be steady and constant in LIO-SEGMOT. It is possible to be resolved by
introducing multiple motion models to LIO-SEGMOT and adaptively selecting proper
models during the factor graph optimization, in which the concept is similar to
an interacting multiple model (IMM) in filtering-based object tracking
approaches.

### Possible Future Research Directions

There are two possible future research directions of LIO-SEGMOT:

- The optimization problem of LIO-SEGMOT produces a multi-robot architecturethat
  may break the efficiency of maintaining single root Bayes trees in iSAM2. It
  causes an unignorable computational cost, especially when there are lots of
  dynamic objects coupled in the system. In forthcoming researches, we would
  like to overcome the bottleneck by introducing the
  [multi-robot iSAM2 (MR-iSAM2)](http://doi.org/10.1109/iros51168.2021.9636687)
  algorithm.
- In addition, rule-based coupling conditions make the proposed method lack the
  ability to explore global optimality when considering combinatorial ambiguity
  as unknown integer variables. It raises a complicated mix-integer programming
  (MIP) problem, whereas a recent work called
  [multi-hypothesis iSAM (MH-iSAM2)](https://www.cs.cmu.edu/~kaess/pub/Hsiao19icra.pdf)
  still promotes us to investigate the challenging problem in the future.

## :gift: Acknowledgement

The project is mainly developed based on [Tixiao
Shan](https://github.com/TixiaoShan)'s excellent work
[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), which helps me a lot in
constructing LIO-SEGMOT. I would like to express my sincere thanks first. In
addition, I would like to thank [Wu Zheng](https://github.com/Vegeta2020) and
[Livox](https://github.com/Livox-SDK) for developing and releasing their awesome
object detection modules [SE-SSD](https://github.com/Vegeta2020/SE-SSD) and
[Livox Detection](https://github.com/Livox-SDK/livox_detection) respectively.
