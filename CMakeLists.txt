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

find_package(OpenCV 3 REQUIRED)

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

add_library(movementlibrary src/Movement.cpp)
add_library(goodobjectlibrary src/GoodObject.cpp)
add_library(objectlibrary src/ObjectList.cpp src/GoodObject.cpp)

find_package(rostest REQUIRED)
## Add gtest based cpp test target and link libraries

catkin_add_gtest(movement_test test/MovementTest.cpp)
if(TARGET movement_test)
  target_link_libraries(movement_test movementlibrary ${catkin_LIBRARIES})
endif()

catkin_add_gtest(goodobject_test test/GoodObjectTest.cpp)
if(TARGET goodobject_test)
  target_link_libraries(goodobject_test goodobjectlibrary ${catkin_LIBRARIES})
endif()

catkin_add_gtest(objectlist_test test/ObjectListTest.cpp)
if(TARGET objectlist_test)
  target_link_libraries(objectlist_test objectlibrary ${catkin_LIBRARIES})
endif()

