cmake_minimum_required(VERSION 3.0.2)
project(mrmf_core)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    ifopt
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
)

include_directories( 
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
    src/unique_id.cpp
    src/robot.cpp
    src/multi_robot_group.cpp
    src/waypoint.cpp
    src/cartesian_trajectory.cpp
    src/sync_trajectory.cpp
    src/composite_interpolator.cpp
    src/sync_trajectory_planner.cpp
    src/positioner_opt.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${catkin_LIBRARIES})