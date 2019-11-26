cmake_minimum_required(VERSION 2.8.3)
project(pac-man)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  kobuki_msgs
  roscpp
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# add_message_files(
#   FILES
# )

# add_service_files(
#   FILES
# )

# add_action_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   kobuki_msgs#   sensor_msgs#   std_msgs
# )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories( include 
                     ${catkin_INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS} )

add_executable(movement src/MovementNode.cpp src/Movement.cpp)
target_link_libraries(movement ${catkin_LIBRARIES})
add_executable(objectlist src/ObjectListNode.cpp src/ObjectList.cpp)
target_link_libraries(objectlist ${catkin_LIBRARIES})
add_executable(object src/ObjectNode.cpp src/Object.cpp src/GoodObject.cpp src/BadObject.cpp)
target_link_libraries(object ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pac-man.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
