
add_executable(ukf ahrs.cpp ahrs.h main.cpp)
add_dependencies(ukf igvc_msgs_gencpp)
target_link_libraries(ukf ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

