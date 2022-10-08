cmake_minimum_required(VERSION 3.0)

project(useful_examples)

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "read_bit: " ${PROJECT_SOURCE_DIR})


add_executable(read_bit read_bit.cpp)

add_executable(read_bag read_bag.cpp)

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
target_include_directories(read_bag
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

message("catkin_LIBRARIES=================${catkin_LIBRARIES}")
target_link_libraries(read_bag
	${Boost_LIBRARY}
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )


#------------------------------------
add_executable(sub_pc sub_pc.cpp)

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
target_include_directories(sub_pc
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(sub_pc
	${Boost_LIBRARY}
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

#------------------------------------
add_executable(sub_imu sub_imu.cpp)

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
target_include_directories(sub_imu
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(sub_imu
	${Boost_LIBRARY}
	${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )