cmake_minimum_required(VERSION 2.8.3)
project(lvi_sam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wno-error=deprecated-declarations)

################## 编译开关 compile switch##############
# -DIF_OFFICIAL=1: use origin official LVI-SAM code
# -DIF_OFFICIAL=0: use modified code of this repo
add_definitions(-DIF_OFFICIAL=0)


######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
#set(OpenCV_DIR "/LVI-SAM/opencv-4.8.1/build/installed")

######################
### Packages
######################
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    roslib
    # msg
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    # cv
    cv_bridge
    # pcl
    pcl_conversions
    # msg generation
    message_generation
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system timer serialization thread)


######################
### Message generation
######################
add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

######################
### Catkin
######################
catkin_package(
    DEPENDS PCL GTSAM
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)

######################
### visual odometry
######################
file(GLOB visual_feature_files
    "src/visual_odometry/visual_feature/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)
file(GLOB visual_odometry_files
    "src/visual_odometry/visual_estimator/*.cpp"
    "src/visual_odometry/visual_estimator/factor/*.cpp"
    "src/visual_odometry/visual_estimator/initial/*.cpp"
    "src/visual_odometry/visual_estimator/utility/*.cpp"
)
file(GLOB visual_loop_files
    "src/visual_odometry/visual_loop/*.cpp"
    "src/visual_odometry/visual_loop/utility/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DBoW/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DUtils/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DVision/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)
# Visual Feature Tracker
add_executable(${PROJECT_NAME}_visual_feature ${visual_feature_files})
target_link_libraries(${PROJECT_NAME}_visual_feature ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
# Visual Odometry
add_executable(${PROJECT_NAME}_visual_odometry ${visual_odometry_files})
target_link_libraries(${PROJECT_NAME}_visual_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
# Visual Lopp
add_executable(${PROJECT_NAME}_visual_loop ${visual_loop_files})
target_link_libraries(${PROJECT_NAME}_visual_loop ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 

######################
### lidar odometry
######################

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Boost::timer Boost::serialization Boost::thread)
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/lidar_odometry/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/lidar_odometry/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/lidar_odometry/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam Boost::timer Boost::serialization Boost::thread)

# Scan-to-map minimal node (LOAM-style optimizer)
add_executable(${PROJECT_NAME}_scan_to_map src/lidar_odometry/scan_to_map.cpp)
add_dependencies(${PROJECT_NAME}_scan_to_map ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_scan_to_map PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_scan_to_map PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam Boost::timer Boost::serialization Boost::thread)
