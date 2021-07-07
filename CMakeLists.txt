cmake_minimum_required(VERSION 3.4...3.18)
project(orbslam)

add_subdirectory(extern/pybind11)
add_subdirectory(extern/ORB_SLAM3)

find_package(OpenCV)

include_directories(
extern/ORB_SLAM3
extern/ORB_SLAM3/include
extern/ORB_SLAM3/include/CameraModels
${OpenCV_INCLUDE_DIRS}
src/
)

link_directories("C:\\local\\boost_1_75_0\\lib64-msvc-14.2")
MESSAGE("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})
pybind11_add_module(orbslam src/slam.cc src/cvt.cpp)
link_directories(extern/ORB3_pybinding/extern/ORB_SLAM3/lib/Release)
target_link_libraries(orbslam PRIVATE ORB_SLAM3)
# target_link_libraries(orbslam PRIVATE "${PROJECT_SOURCE_DIR}/extern/ORB_SLAM3/lib/Release/ORB_SLAM3.lib")

target_compile_definitions(orbslam PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
