cmake_minimum_required(VERSION 3.13)
project(shortest_path)
set(CMAKE_CXX_STANDARD 14)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.71 REQUIRED)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(SOURCE_FILES test_astar_gridmap.cpp)
add_executable(test_astar_gridmap ${SOURCE_FILES})
target_link_libraries(test_astar_gridmap ${OpenCV_LIBS} )