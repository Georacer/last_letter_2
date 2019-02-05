cmake_minimum_required(VERSION 3.10.2)


project(last_letter_2)


find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  geometry_msgs
  rosgraph_msgs
  std_msgs
  std_srvs
  message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

add_message_files(
  FILES
  joystick_input.msg
  input_signals.msg
  model_wrenches.msg
  model_states.msg
  air_data.msg
  control_signals.msg
  aero_wrenches.msg
  prop_wrenches.msg
)

add_service_files(
  FILES
  apply_wrench_srv.srv
  get_model_states_srv.srv
  get_control_signals_srv.srv
  airdata_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(DEPENDS
  CATKIN_DEPENDS  message_runtime
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library(model_plugin src/gazebo/model_plugin.cpp)
add_library(world_plugin SHARED src/gazebo/world_plugin.cpp)


add_executable(joy2chan src/ros/joy2chan.cpp) 
add_executable(chan2basics src/ros/chan2basics.cpp) 
add_executable(core src/ros/core.cpp)
add_executable(controller_node src/ros/controller_node.cpp)
add_executable(environment src/ros/environment.cpp)

add_dependencies(joy2chan ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp last_letter_2_gencpp)
add_dependencies(chan2basics ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp last_letter_2_gencpp)
add_dependencies(model_plugin ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(world_plugin ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(controller_node ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp last_letter_2_gencpp)
add_dependencies(core ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp last_letter_2_gencpp)
add_dependencies(environment ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_generate_messages_cpp last_letter_2_gencpp)

target_link_libraries(joy2chan ${catkin_LIBRARIES})
target_link_libraries(chan2basics ${catkin_LIBRARIES})
target_link_libraries(model_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(core ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(controller_node ${catkin_LIBRARIES})
target_link_libraries(environment ${catkin_LIBRARIES})



