cmake_minimum_required(VERSION 3.0.2)
project(kaist2bag)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
        rosbag
        image_transport
        cv_bridge
        pcl_ros
        pcl_conversions
        pcl_msgs
        irp_sen_msgs
        tf
)

find_package(Boost REQUIRED COMPONENTS
        filesystem)

find_package(OpenCV REQUIRED)

catkin_package(
    LIBRARIES kaist2bag
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
    rosbag
    image_transport
    cv_bridge
    pcl_ros
    pcl_conversions
    pcl_msgs
    irp_sen_msgs
    tf
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/converter.cpp
        src/altimeter_converter.cpp
        src/encoder_converter.cpp
        src/fog_converter.cpp
        src/gps_converter.cpp
        src/vrs_converter.cpp
        src/imu_converter.cpp
        src/velodyne_converter.cpp
        src/sick_converter.cpp
        src/stereo_converter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARY_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/mergebag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



