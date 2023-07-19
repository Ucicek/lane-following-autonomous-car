cmake_minimum_required(VERSION 3.5)
project(line_following_robot VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

pybind11_add_module(lane_detection lane_detection.cpp detect_lanes.cpp)
target_link_libraries(lane_detection PRIVATE ${OpenCV_LIBS})