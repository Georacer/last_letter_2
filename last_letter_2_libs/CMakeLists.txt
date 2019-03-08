cmake_minimum_required(VERSION 3.10.2)
project(last_letter_2_libs)


find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  rosgraph_msgs
  std_srvs
  geometry_msgs
)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES last_letter_2_libs
)

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR}
)

add_library(last_letter_2_libs src/last_letter_2_libs.cpp)

target_link_libraries(last_letter_2_libs ${catkin_LIBRARIES})


install(TARGETS last_letter_2_libs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN ".h" PATTERN ".hpp"
  PATTERN ".svn" EXCLUDE)