cmake_minimum_required(VERSION 3.0.2)
project(map_change_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  message_generation
  cv_bridge
  image_transport
  tf
)
find_package(OpenCV REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ChangedCells.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  UpdateMap.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES map_change_detection grid
 CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs geometry_msgs tf2 tf2_ros message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(map_change_detection
  src/map_change_detection.cpp
)

add_library(scan_handler
  src/scan_handler.cpp
)

add_library(grid
  src/grid.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/map_updater_node.cpp)
add_executable(${PROJECT_NAME}_processer_node src/map_processer_node.cpp)
add_executable(image_publisher src/image_publisher.cpp )
target_link_libraries(image_publisher ${catkin_LIBRARIES})

add_dependencies(grid ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}_processer_node ${PROJECT_NAME}_gencpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  map_change_detection
  scan_handler
  grid
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_processer_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_map_change_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
