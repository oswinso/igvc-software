add_executable(madgwick madgwick_filter_node.h madgwick_filter_node.cpp madgwick_filter.cpp madgwick_filter.h)
add_dependencies(madgwick igvc_msgs_gencpp)
target_link_libraries(madgwick ${catkin_LIBRARIES} ${PCL_LIBRARIES})
