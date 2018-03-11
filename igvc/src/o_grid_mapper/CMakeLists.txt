add_executable(o_grid_mapper main.cpp)
add_dependencies(o_grid_mapper igvc_msgs_gencpp)
include_directories(${OCTOMAP_INCLUDE_DIRS})

target_link_libraries(o_grid_mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})