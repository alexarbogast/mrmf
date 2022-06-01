set(MRMF_LIB_NAME mrmf_interface)

add_library(${MRMF_LIB_NAME} src/mrmf_interface.cpp)
target_link_libraries(${MRMF_LIB_NAME} ${catkin_LIBRARIES})