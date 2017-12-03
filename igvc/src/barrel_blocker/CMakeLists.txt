add_executable(barrel_blocker main.cpp)
add_dependencies(barrel_blocker igvc_msgs_gencpp)
target_link_libraries(barrel_blocker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
