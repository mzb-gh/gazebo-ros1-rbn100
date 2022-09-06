cmake_minimum_required(VERSION 2.8.3)
project(rbn100_gazebo_plugins)

# gazebo的和ros的
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS gazebo_ros
                                        gazebo_plugins
                                        geometry_msgs
                                        rbn100_msgs
                                        nav_msgs
                                        roscpp
                                        sensor_msgs
                                        std_msgs
                                        tf)

catkin_package(INCLUDE_DIRS include
               LIBRARIES gazebo_ros_rbn100 
               CATKIN_DEPENDS gazebo_ros
                              gazebo_plugins
                              geometry_msgs
                              rbn100_msgs
                              nav_msgs
                              roscpp
                              sensor_msgs
                              std_msgs
                              tf)

# 编译gazebo插件需指定c++标志
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})

# plugins
add_library(gazebo_ros_rbn100 src/gazebo_ros_rbn100.cpp
                              src/gazebo_ros_rbn100_updates.cpp
                              src/gazebo_ros_rbn100_loads.cpp)
add_dependencies(gazebo_ros_rbn100 ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_rbn100
                      ${catkin_LIBRARIES}
                      ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_ros_rbn100
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# 制作好插件就可以和urdf（sdf）中的元素绑定
