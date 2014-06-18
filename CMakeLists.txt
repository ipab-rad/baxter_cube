cmake_minimum_required(VERSION 2.8.3)
project(baxter_cube)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  baxter_core_msgs
  #baxter_interface
  #baxter_description
  control_msgs
  sensor_msgs
  #trajectory_msgs
  #geometry_msgs
  #std_msgs
  #tf
  image_transport
  cv_bridge
  #visp_bridge
  #moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(VISP REQUIRED)
if(VISP_FOUND)
	add_definitions(${VISP_DEFINITIONS})
	include_directories(${VISP_INCLUDE_DIRS})
	link_directories(${VISP_LIBRARY_DIRS})
	link_libraries(${VISP_LIBRARIES})
endif(VISP_FOUND)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_demo
CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(baxter_demo
#   src/${PROJECT_NAME}/baxter_demo.cpp
# )

## Declare a cpp executable
#add_executable(model_tracker src/model_tracker.cpp)
#add_executable(image_processing src/test.cpp)
#add_executable(image_processing src/demo.cpp)
#add_executable(camera_velocity src/camera_velocity_estimation.cpp)
#add_executable(joint_velocities src/joints_velocity_estimation.cpp)
#add_executable(left_camera src/left_camera.cpp)
add_executable(right_camera src/right_camera.cpp)
add_executable(state_machine src/state_machine.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(model_tracker baxter_demo_generate_messages_cpp)
#add_dependencies(image_processing baxter_demo_generate_messages_cpp)
#add_dependencies(camera_velocity baxter_demo_generate_messages_cpp)
#add_dependencies(joint_velocities baxter_demo_generate_messages_cpp)
#add_dependencies(left_camera baxter_demo_generate_messages_cpp)
add_dependencies(right_camera baxter_demo_generate_messages_cpp)
add_dependencies(state_machine baxter_demo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(model_tracker ${catkin_LIBRARIES})
#target_link_libraries(image_processing ${catkin_LIBRARIES})
#target_link_libraries(camera_velocity ${catkin_LIBRARIES})
#target_link_libraries(joint_velocities ${catkin_LIBRARIES})
#target_link_libraries(left_camera ${catkin_LIBRARIES})
target_link_libraries(right_camera ${catkin_LIBRARIES})
target_link_libraries(state_machine ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS baxter_demo baxter_demo_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_baxter_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
