#include "utility.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

std::queue<std::vector<rosbag::MessageInstance>> patchedInstances;

float base_rate;
std::string bag_filename;
std::string imu_topic;
std::string lidar_topic;
std::string pub_imu_topic;
std::string pub_lidar_topic;

ros::Publisher pubImu;
ros::Publisher pubLiDAR;

ros::Time currentTime;

void playInstances(const std::vector<rosbag::MessageInstance>& instances) {
  for (auto& instance : instances) {
    auto duration = instance.getTime() - currentTime;
    if (duration.toSec() > 0) {
      ros::WallDuration(duration.toSec() / base_rate).sleep();
    }

    auto imuMsg = instance.instantiate<sensor_msgs::Imu>();
    if (imuMsg != nullptr) {
      pubImu.publish(imuMsg);
    }
    auto lidarMsg = instance.instantiate<sensor_msgs::PointCloud2>();
    if (lidarMsg != nullptr) {
      pubLiDAR.publish(lidarMsg);
    }

    currentTime = instance.getTime();
  }
}

void odometryIsDoneCallback(const std_msgs::EmptyConstPtr& msg) {
  playInstances(patchedInstances.front());
  patchedInstances.pop();
  ROS_INFO("Remaining patches: %lu", patchedInstances.size());
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "offline_bag_player");
  ros::NodeHandle _nh("~");
  ros::NodeHandle nh;

  _nh.param<float>("base_rate", base_rate, 1.0);
  _nh.param<std::string>("bag_filename", bag_filename, "");
  _nh.param<std::string>("imu_topic", imu_topic, "/imu_raw");
  _nh.param<std::string>("lidar_topic", lidar_topic, "/points_raw");
  _nh.param<std::string>("pub_imu_topic", pub_imu_topic, "/imu_raw");
  _nh.param<std::string>("pub_lidar_topic", pub_lidar_topic, "/points_raw");

  ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("lio_segmot/ready", 10, &odometryIsDoneCallback);

  pubImu   = nh.advertise<sensor_msgs::Imu>(pub_imu_topic, 2000);
  pubLiDAR = nh.advertise<sensor_msgs::PointCloud2>(pub_lidar_topic, 1);

  if (bag_filename.empty()) {
    ROS_ERROR("bag_filename is empty");
    return -1;
  }

  // Read and process the bag file
  rosbag::Bag bag;
  ROS_INFO("Opening bag file: %s", bag_filename.c_str());
  bag.open(bag_filename, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(imu_topic);
  topics.push_back(lidar_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<rosbag::MessageInstance> instances;
  bool timeIsInitialized = false;
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    instances.push_back(m);
    if (m.getTopic() == lidar_topic) {
      patchedInstances.push(instances);
      instances.clear();
    }

    if (!timeIsInitialized) {
      currentTime       = m.getTime();
      timeIsInitialized = true;
    }
  }

  // Make sure that the system is ready to subscribe the data
  ROS_INFO("Pending ...");
  while (pubImu.getNumSubscribers() < 2 || pubLiDAR.getNumSubscribers() < 1) {
    ros::spinOnce();
  }

  ROS_INFO("Start playing the bag ...");
  // Play the first patch of the data
  playInstances(patchedInstances.front());
  patchedInstances.pop();

  while (!patchedInstances.empty() && ros::ok()) {
    ros::spinOnce();
  }

  bag.close();

  return 0;
}