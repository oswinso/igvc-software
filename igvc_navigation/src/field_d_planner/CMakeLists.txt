find_package(OpenMP)
if (OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/FindGperftools.cmake)

include_directories(
  ${GPERFTOOLS_INCLUDE_DIR}
)
set(Gperftools_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake/")
find_package(Gperftools REQUIRED)

# set the path to the library folder
link_directories(/usr/local/lib)

add_executable(field_d_planner
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Graph.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Node.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/PriorityQueue.cpp
        FieldDPlanner.cpp
        main.cpp)
add_dependencies(field_d_planner igvc_msgs_gencpp)
target_link_libraries(field_d_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GPERFTOOLS_LIBRARIES})
