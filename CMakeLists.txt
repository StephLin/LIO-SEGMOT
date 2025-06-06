cmake_minimum_required(VERSION 2.8.3)
project(lio_segmot)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g0 -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  rosbag
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  jsk_recognition_msgs
  jsk_topic_tools
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  ObjectState.msg
  ObjectStateArray.msg
  Diagnosis.msg
  flags.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
  detection.srv
  save_estimation_result.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  jsk_recognition_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  jsk_recognition_msgs
  jsk_topic_tools
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptimization src/mapOptimization.cpp src/factor.cpp src/solver.cpp)
add_dependencies(${PROJECT_NAME}_mapOptimization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptimization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptimization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# Offline ROSBag Player
add_executable(${PROJECT_NAME}_offlineBagPlayer src/offlineBagPlayer.cpp)
target_link_libraries(${PROJECT_NAME}_offlineBagPlayer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)
