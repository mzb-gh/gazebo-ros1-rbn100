cmake_minimum_required(VERSION 3.0)

project(mind_os VERSION 1.0)

add_definitions(-std=c++17)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# add_subdirectory(src/nodes/core)
# add_subdirectory(src/nodes/publisher)
# add_subdirectory(src/nodes/subscriber)

find_package(catkin REQUIRED 
             COMPONENTS geometry_msgs
                        roscpp
                        geometry_msgs
                        std_msgs
                        tf
                        nav_msgs
                        sensor_msgs
                        std_srvs
                        rospack
                        roslib)

include_directories(include
                    ../devel/include
                    ${catkin_INCLUDE_DIRS})

add_library(ros_plugin SHARED src/RosNode.cpp)
target_link_libraries(ros_plugin
                      pthread
                      boost_system
                      glog
                      mind_os
                      ${catkin_LIBRARIES})

add_library(ros_subscriber_plugin SHARED src/RosSubscriberNode.cpp)
target_link_libraries(ros_subscriber_plugin
                      pthread
                      boost_system
                      glog
                      mind_os
                      ${catkin_LIBRARIES})

# add_library(ros_publisher_plugin SHARED src/RosPublisherNode.cpp)
# target_link_libraries(ros_publisher_plugin
#                       pthread
#                       boost_system
#                       glog
#                       mind_os
#                       ${catkin_LIBRARIES})
