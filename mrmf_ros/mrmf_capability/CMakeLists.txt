set(MRMF_LIB_NAME mrmf_capability)

add_library(${MRMF_LIB_NAME} src/mrmf_test_service_capability.cpp src/synchronous_trajectory_service_capability.cpp)
target_link_libraries(${MRMF_LIB_NAME} ${catkin_LIBRARIES})