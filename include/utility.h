#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/impl/search.hpp>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE,
                        OUSTER };

class ParamServer {
 public:
  ros::NodeHandle nh;

  std::string robot_id;

  // Topics
  string pointCloudTopic;
  string imuTopic;
  string odomTopic;
  string gpsTopic;

  // Frames
  string lidarFrame;
  string baselinkFrame;
  string odometryFrame;
  string mapFrame;

  // GPS Settings
  bool useImuHeadingInitialization;
  bool useGpsElevation;
  float gpsCovThreshold;
  float poseCovThreshold;

  // Save pcd
  bool savePCD;
  string savePCDDirectory;

  // Lidar Sensor Configuration
  SensorType sensor;
  int N_SCAN;
  int Horizon_SCAN;
  int downsampleRate;
  float lidarMinRange;
  float lidarMaxRange;

  // IMU
  float imuAccNoise;
  float imuGyrNoise;
  float imuAccBiasN;
  float imuGyrBiasN;
  float imuGravity;
  float imuRPYWeight;
  vector<double> extRotV;
  vector<double> extRPYV;
  vector<double> extTransV;
  Eigen::Matrix3d extRot;
  Eigen::Matrix3d extRPY;
  Eigen::Vector3d extTrans;
  Eigen::Quaterniond extQRPY;

  // LOAM
  float edgeThreshold;
  float surfThreshold;
  int edgeFeatureMinValidNum;
  int surfFeatureMinValidNum;

  // voxel filter paprams
  float odometrySurfLeafSize;
  float mappingCornerLeafSize;
  float mappingSurfLeafSize;

  float z_tollerance;
  float rotation_tollerance;

  // CPU Params
  int numberOfCores;
  double mappingProcessInterval;

  // Surrounding map
  float surroundingkeyframeAddingDistThreshold;
  float surroundingkeyframeAddingAngleThreshold;
  float surroundingKeyframeDensity;
  float surroundingKeyframeSearchRadius;

  // Loop closure
  bool loopClosureEnableFlag;
  float loopClosureFrequency;
  int surroundingKeyframeSize;
  float historyKeyframeSearchRadius;
  float historyKeyframeSearchTimeDiff;
  int historyKeyframeSearchNum;
  float historyKeyframeFitnessScore;

  // global map visualization radius
  float globalMapVisualizationSearchRadius;
  float globalMapVisualizationPoseDensity;
  float globalMapVisualizationLeafSize;

  // Dynamic object data association
  float detectionMatchThreshold;
  vector<double> dataAssociationVarianceVector;
  vector<double> earlyLooselyCoupledMatchingVarianceVector;
  vector<double> looselyCoupledMatchingVarianceVector;
  vector<double> tightlyCoupledMatchingVarianceVector;

  Eigen::Matrix<double, 6, 1> dataAssociationVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> earlyLooselyCoupledMatchingVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> looselyCoupledMatchingVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> tightlyCoupledMatchingVarianceEigenVector;

  // Factor covariance matrices (presented as diagonal vectors)
  vector<double> priorOdometryDiagonalVarianceVector;
  vector<double> odometryDiagonalVarianceVector;
  vector<double> earlyConstantVelocityDiagonalVarianceVector;
  vector<double> constantVelocityDiagonalVarianceVector;
  vector<double> motionDiagonalVarianceVector;
  vector<double> looselyCoupledDetectionVarianceVector;
  vector<double> tightlyCoupledDetectionVarianceVector;

  Eigen::Matrix<double, 6, 1> priorOdometryDiagonalVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> odometryDiagonalVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> earlyConstantVelocityDiagonalVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> constantVelocityDiagonalVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> motionDiagonalVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> looselyCoupledDetectionVarianceEigenVector;
  Eigen::Matrix<double, 6, 1> tightlyCoupledDetectionVarianceEigenVector;

  // Gentle coupling options
  int numberOfEarlySteps;
  int numberOfPreLooseCouplingSteps;
  int numberOfInterLooseCouplingSteps;
  float tightCouplingDetectionErrorThreshold;

  float objectIsMovingFastThreshold;
  float objectIsTurningThreshold;

  // Tracking
  int trackingStepsForLostObject;

  ParamServer() {
    nh.param<std::string>("/robot_id", robot_id, "roboat");

    nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
    nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
    nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
    nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

    nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
    nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
    nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
    nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

    nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
    nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
    nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

    nh.param<bool>("lio_sam/savePCD", savePCD, false);
    nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

    std::string sensorStr;
    nh.param<std::string>("lio_sam/sensor", sensorStr, "");
    if (sensorStr == "velodyne") {
      sensor = SensorType::VELODYNE;
    } else if (sensorStr == "ouster") {
      sensor = SensorType::OUSTER;
    } else {
      ROS_ERROR_STREAM(
          "Invalid sensor type (must be either 'velodyne' or 'ouster'): " << sensorStr);
      ros::shutdown();
    }

    nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
    nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
    nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
    nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
    nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

    nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
    nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
    nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
    nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
    nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
    nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
    nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
    nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
    nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
    extRot   = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extRPY   = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    extQRPY  = Eigen::Quaterniond(extRPY);

    nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
    nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
    nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
    nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

    nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
    nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
    nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

    nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
    nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

    nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
    nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

    nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
    nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

    nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

    nh.param<float>("lio_sam/detectionMatchThreshold", detectionMatchThreshold, 19.5);
    nh.param<vector<double>>("lio_sam/dataAssociationVarianceVector", dataAssociationVarianceVector, {1e-4, 1e-4, 1e-4, 1e-2, 2e-3, 2e-3});
    nh.param<vector<double>>("lio_sam/earlyLooselyCoupledMatchingVarianceVector", earlyLooselyCoupledMatchingVarianceVector, {1e-4, 1e-4, 1e-4, 1e-2, 2e-3, 2e-3});
    nh.param<vector<double>>("lio_sam/looselyCoupledMatchingVarianceVector", looselyCoupledMatchingVarianceVector, {1e-4, 1e-4, 1e-4, 1e-2, 2e-3, 2e-3});
    nh.param<vector<double>>("lio_sam/tightlyCoupledMatchingVarianceVector", tightlyCoupledMatchingVarianceVector, {1e-4, 1e-4, 1e-4, 1e-2, 2e-3, 2e-3});

    nh.param<vector<double>>("lio_sam/priorOdometryDiagonalVarianceVector", priorOdometryDiagonalVarianceVector, {1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8});
    nh.param<vector<double>>("lio_sam/odometryDiagonalVarianceVector", odometryDiagonalVarianceVector, {1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4});
    nh.param<vector<double>>("lio_sam/earlyConstantVelocityDiagonalVarianceVector", earlyConstantVelocityDiagonalVarianceVector, {1e-3, 1e-3, 1e-3, 2e-1, 1e-1, 1e-1});
    nh.param<vector<double>>("lio_sam/constantVelocityDiagonalVarianceVector", constantVelocityDiagonalVarianceVector, {1e-3, 1e-3, 1e-3, 2e-1, 1e-1, 1e-1});
    nh.param<vector<double>>("lio_sam/motionDiagonalVarianceVector", motionDiagonalVarianceVector, {1e-4, 1e-4, 1e-2, 1e-1, 1e-2, 1e-2});
    nh.param<vector<double>>("lio_sam/looselyCoupledDetectionVarianceVector", looselyCoupledDetectionVarianceVector, {1e-4, 1e-4, 1e-4, 1e-2, 2e-3, 2e-3});
    nh.param<vector<double>>("lio_sam/tightlyCoupledDetectionVarianceVector", tightlyCoupledDetectionVarianceVector, {1e-4, 1e-4, 1e-4, 1e-2, 2e-3, 2e-3});

    dataAssociationVarianceEigenVector             = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(dataAssociationVarianceVector.data());
    earlyLooselyCoupledMatchingVarianceEigenVector = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(earlyLooselyCoupledMatchingVarianceVector.data());
    looselyCoupledMatchingVarianceEigenVector      = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(looselyCoupledMatchingVarianceVector.data());
    tightlyCoupledMatchingVarianceEigenVector      = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(tightlyCoupledMatchingVarianceVector.data());

    priorOdometryDiagonalVarianceEigenVector         = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(priorOdometryDiagonalVarianceVector.data());
    odometryDiagonalVarianceEigenVector              = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(odometryDiagonalVarianceVector.data());
    earlyConstantVelocityDiagonalVarianceEigenVector = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(earlyConstantVelocityDiagonalVarianceVector.data());
    constantVelocityDiagonalVarianceEigenVector      = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(constantVelocityDiagonalVarianceVector.data());
    motionDiagonalVarianceEigenVector                = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(motionDiagonalVarianceVector.data());
    looselyCoupledDetectionVarianceEigenVector       = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(looselyCoupledDetectionVarianceVector.data());
    tightlyCoupledDetectionVarianceEigenVector       = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(tightlyCoupledDetectionVarianceVector.data());

    nh.param<int>("lio_sam/numberOfEarlySteps", numberOfEarlySteps, 1);
    nh.param<int>("lio_sam/numberOfPreLooseCouplingSteps", numberOfPreLooseCouplingSteps, 10);
    nh.param<int>("lio_sam/numberOfInterLooseCouplingSteps", numberOfInterLooseCouplingSteps, 0);
    nh.param<float>("lio_sam/tightCouplingDetectionErrorThreshold", tightCouplingDetectionErrorThreshold, 500.0);

    nh.param<float>("lio_sam/objectIsMovingFastThreshold", objectIsMovingFastThreshold, 1.0);
    nh.param<float>("lio_sam/objectIsTurningThreshold", objectIsTurningThreshold, 30);

    nh.param<int>("lio_sam/trackingStepsForLostObject", trackingStepsForLostObject, 3);

    usleep(100);
  }

  sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc                           = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr                        = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x      = q_final.x();
    imu_out.orientation.y      = q_final.y();
    imu_out.orientation.z      = q_final.z();
    imu_out.orientation.w      = q_final.w();

    if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1) {
      ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
      ros::shutdown();
    }

    return imu_out;
  }
};

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame) {
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*thisCloud, tempCloud);
  tempCloud.header.stamp    = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  if (thisPub->getNumSubscribers() != 0)
    thisPub->publish(tempCloud);
  return tempCloud;
}

template <typename T>
double ROS_TIME(T msg) {
  return msg->header.stamp.toSec();
}

template <typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z) {
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {
  double imuRoll, imuPitch, imuYaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  *rosRoll  = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw   = imuYaw;
}

float pointDistance(PointType p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

#endif
