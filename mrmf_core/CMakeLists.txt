cmake_minimum_required(VERSION 3.0.2)
project(mrmf_core)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    bio_ik
)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        bio_ik
)

include_directories( 
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/goal_types.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})