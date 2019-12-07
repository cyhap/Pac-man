cmake_minimum_required(VERSION 2.8.3)
project(pacman)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  kobuki_msgs
  roscpp
  sensor_msgs
  std_msgs
  gazebo_msgs
  gazebo_ros
  message_generation
  pcl_conversions
  pcl_ros
)

################################################
## Declare ROS messages, services and actions ##
################################################

 add_message_files(
   FILES
   ObjPose.msg
   VecPoses.msg
 )

 add_service_files(
  FILES
  NavPose.srv
 )

# add_action_files(
#   FILES
# )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

find_package(OpenCV 3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

include_directories( include 
                     ${catkin_INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS} )

add_executable(movement src/MovementNode.cpp src/Movement.cpp)
target_link_libraries(movement ${catkin_LIBRARIES})
add_dependencies(movement pacman_generate_messages_cpp)

add_executable(objectlist src/ObjectListNode.cpp src/ObjectList.cpp)
target_link_libraries(objectlist ${catkin_LIBRARIES})

add_executable(pubpose src/pubpose.cpp)
target_link_libraries(pubpose ${catkin_LIBRARIES})
add_dependencies(pubpose pacman_generate_messages_cpp)

add_executable(object src/IdentificationNode.cpp src/GoodObject.cpp src/BadObject.cpp src/ImageProcessing.cpp)
target_link_libraries(object ${catkin_LIBRARIES})
add_dependencies(object pacman_generate_messages_cpp)

add_executable(navigation src/NavigationNode.cpp)
target_link_libraries(navigation ${catkin_LIBRARIES})
add_dependencies(navigation pacman_generate_messages_cpp)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  add_library(movementlibrary src/Movement.cpp)
  add_library(objectlibrary src/GoodObject.cpp src/BadObject.cpp)
  add_library(objectlistlibrary src/ObjectList.cpp src/GoodObject.cpp)
  add_library(imageProcesinglibrary src/ImageProcessing.cpp)

  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(movement_test test/MovementTest.cpp)
  target_link_libraries(movement_test movementlibrary ${catkin_LIBRARIES})

  catkin_add_gtest(objects_test test/ObjectTest.cpp)
  target_link_libraries(objects_test objectlibrary ${catkin_LIBRARIES})

  catkin_add_gtest(objectlist_test test/ObjectListTest.cpp)
  target_link_libraries(objectlist_test objectlistlibrary ${catkin_LIBRARIES})

  catkin_add_gtest(imageProcessing_test test/ImageProcessingTest.cpp)
  target_link_libraries(imageProcessing_test imageProcesinglibrary objectlibrary ${catkin_LIBRARIES})
endif()
