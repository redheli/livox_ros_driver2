cmake_minimum_required(VERSION 3.0)

project(read_bit)

set(CMAKE_CXX_STANDARD 11)

message(STATUS "read_bit: " ${PROJECT_SOURCE_DIR})


add_executable(read_bit read_bit.cpp)

# target_link_libraries(${DEMO_NAME}
#         PUBLIC
#         livox_lidar_sdk_static
# 				)



