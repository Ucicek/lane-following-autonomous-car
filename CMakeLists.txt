cmake_minimum_required(VERSION 3.0.0)
project(line_following_robot VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories((src/include))

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED)

add_executable(line_following_robot main.cpp  src/include/detec_lanes.cpp)
target_link_libraries(line_following_robot ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})