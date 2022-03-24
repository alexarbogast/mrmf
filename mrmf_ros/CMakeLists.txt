cmake_minimum_required(VERSION 3.0.2)
project(mrmf_ros)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    roscpp
)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(mrmf_group src/mrmf_group.cpp)
target_link_libraries(mrmf_group ${catkin_LIBRARIES})

add_library(mrmf_group_interface src/mrmf_group_interface.cpp)
target_link_libraries(mrmf_group_interface ${catkin_LIBRARIES})