cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2)

find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  geometry_msgs
  rosgraph_msgs
  sensor_msgs
  std_msgs
  last_letter_2_msgs
  last_letter_2_libs
  tf2_ros
)
find_package(Boost REQUIRED)

catkin_package(DEPENDS
  CATKIN_DEPENDS 
  std_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  include/propulsion
 ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(joy2chan src/joy2chan.cpp) 
add_executable(chan2basics src/chan2basics.cpp) 
add_executable(core src/core.cpp)
add_executable(controller_mediator src/controller_mediator.cpp)
add_executable(environment src/environment.cpp)
add_executable(controller src/controller.cpp)

add_executable(tf_listener src/tf_listener.cpp)

add_dependencies(joy2chan ${catkin_EXPORTED_TARGETS})
add_dependencies(environment ${catkin_EXPORTED_TARGETS})
add_dependencies(chan2basics ${catkin_EXPORTED_TARGETS})
add_dependencies(core ${catkin_EXPORTED_TARGETS})
add_dependencies(controller_mediator ${catkin_EXPORTED_TARGETS})
add_dependencies(controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(joy2chan ${catkin_LIBRARIES})
target_link_libraries(chan2basics ${catkin_LIBRARIES})
target_link_libraries(core ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(controller_mediator ${catkin_LIBRARIES})
target_link_libraries(environment ${catkin_LIBRARIES})
target_link_libraries(controller ${catkin_LIBRARIES})


target_link_libraries(tf_listener ${catkin_LIBRARIES})



