cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_control)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    unitree_legged_msgs
)

catkin_package(
    CATKIN_DEPENDS
    unitree_legged_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

link_directories($(catkin_LIB_DIRS) lib)

add_library( unitree_legged_control 
    src/joint_controller.cpp
)
target_link_libraries(unitree_legged_control ${catkin_LIBRARIES} unitree_joint_control_tool)
