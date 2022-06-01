cmake_minimum_required(VERSION 3.0.2)
project(mrmf_ros)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    roscpp
)

set(PACKAGE_INCLUDE_DIRS
    mrmf_interface/include
    mrmf_capability/include
)

catkin_package(
    LIBRARIES
        mrmf_interface
        mrmf_capability
    INCLUDE_DIRS
        ${PACKAGE_INCLUDE_DIRS}
    CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_move_group
        roscpp
)

include_directories(${PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_subdirectory(mrmf_interface)
add_subdirectory(mrmf_capability)

install(
  FILES
    planning_request_adapters_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add_executable(mrmf_group src/mrmf_group.cpp)
#target_link_libraries(mrmf_group PUBLIC ${catkin_LIBRARIES})