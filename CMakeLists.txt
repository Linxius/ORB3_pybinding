cmake_minimum_required(VERSION 3.4...3.18)
project(orbslam)

add_subdirectory(extern/pybind11)
add_subdirectory(extern/ORB_SLAM3)

find_package(OpenCV)
# find_package(Eigen3)
# find_package(boost)
# find_package(Pangolin CONFIG REQUIRED)
# find_package(realsense2)

include_directories(
extern/ORB_SLAM3
extern/ORB_SLAM3/include
extern/ORB_SLAM3/include/CameraModels
${OpenCV_INCLUDE_DIRS}
src/
# ${EIGEN3_INCLUDE_DIR}
# ${Pangolin_INCLUDE_DIRS}
# ${Boost_INCLUDE_DIRS}
)
# target_link_libraries(
# ${OpenCV_LIBS}
# ${EIGEN3_LIBS}
# ${Pangolin_LIBRARIES}
# extern/ORB_SLAM3/lib/Release/ORB_SlAM3.lib
# extern/ORB_SLAM3/Thirdparty/g2o/bin/Release/g2o.lib
# extern/ORB_SLAM3/Thirdparty/DBoW2/lib/Release/DBoW2.lib
# )

MESSAGE("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})
# pybind11_add_module(cmake_example src/main.cpp)
pybind11_add_module(orbslam src/slam.cc src/cvt.cpp)
# target_link_libraries(orbslam PRIVATE extern/ORB3_pybinding/extern/ORB_SLAM3/lib/Release/ORB_SLAM3)
link_directories(extern/ORB3_pybinding/extern/ORB_SLAM3/lib/Release)
target_link_libraries(orbslam PRIVATE ORB_SLAM3)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
# target_compile_definitions(cmake_example PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_compile_definitions(orbslam PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
