add_executable(ekf ekf.cpp ekf.h)
add_dependencies(ekf igvc_msgs_gencpp)
target_link_libraries(ekf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${BOOST_LIBRARIES})
