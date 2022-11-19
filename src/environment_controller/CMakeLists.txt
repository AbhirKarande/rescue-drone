cmake_minimum_required(VERSION 3.0.2)
project(environment_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf2
  roscpp
  tf2_ros
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  use_key.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros sensor_msgs
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/environment_controller_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)