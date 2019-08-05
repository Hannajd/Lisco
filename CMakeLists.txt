cmake_minimum_required (VERSION 2.8 FATAL_ERROR)
project (Lisco)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -O3 -std=c++11")

set(${PROJECT_NAME}_headers LinearClustering.hpp LidarSpec.hpp)
set(${PROJECT_NAME}_sources main.cpp LinearClustering.cpp LidarSpec.cpp)
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})