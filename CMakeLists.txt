cmake_minimum_required(VERSION 3.0.2)
project(mpc_ros)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
  tf2
)

catkin_package()
include_directories(
  src/Eigen-3.3
  acado_mpc_export
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/src/acado.h
# )

add_library(ACADO STATIC IMPORTED)

add_executable(mpc_node src/mpc_node.cpp)
set_target_properties(ACADO PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/acado_mpc_export/libacado_exported_rti.a")
target_link_libraries(mpc_node ${catkin_LIBRARIES} ACADO ipopt)

add_executable(test_node src/test_node.cpp)
set_target_properties(ACADO PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/acado_mpc_export/libacado_exported_rti.a")
target_link_libraries(test_node ${catkin_LIBRARIES} ACADO ipopt)

add_executable(path_generator src/make_path.cpp)
target_link_libraries(path_generator ${catkin_LIBRARIES})