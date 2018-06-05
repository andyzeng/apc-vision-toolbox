cmake_minimum_required(VERSION 3.1.0)
set(CMAKE_CXX_STANDARD 11)
project(marvin_convnet)

## Find catkin macros and libraries
## IF COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  std_msgs
  sensor_msgs
)

find_package(OpenCV 2.4 REQUIRED)

# CUDA and CUDNN
find_package(CUDA REQUIRED)
set(CUDA_HOST_COMPILER "c++")
set(CUDA_NVCC_FLAGS "-std=c++11")
set(CUDNN_INCLUDE_DIR /usr/local/cuda/include CACHE PATH "Path to cudnn header file")
set(CUDNN_LIB_DIR /usr/local/cuda/lib64)

find_package(catkin REQUIRED COMPONENTS
    image_transport
    cv_bridge
)

## Uncomment this IF the package has a setup.py. This macro ensures
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
add_message_files(
  FILES
  # ObjectHypothesis.msg
  object_mask.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectObjects.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  marvin_convnet
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this IF you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES apc_vision
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
# )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(apc_vision
#   src/${PROJECT_NAME}/apc_vision.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(apc_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Add C++ header/source files
SET(HEADERS
  include/depth_utils.h
  src/marvin.hpp
)

## Declare a C++ executable
include_directories(include /usr/local/include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR} ${CUDNN_INCLUDE_DIR})
link_directories(${CUDNN_LIB_DIR} ${OpenCV_LIB_DIR})

cuda_add_executable(detect src/detect.cu ${HEADERS})
target_link_libraries(detect ${catkin_LIBRARIES}  ${OpenCV_LIBS}
                      pthread cudnn ${CUDA_CUBLAS_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_curand_LIBRARY})
add_dependencies(detect ${PROJECT_NAME}_gencpp)
add_dependencies(detect ${PROJECT_NAME}_generate_messages_cpp)

#cuda_add_executable(save_images src/save_images.cpp ${HEADERS})
#target_link_libraries(save_images ${catkin_LIBRARIES} realsense ${OpenCV_LIBS}
#                      pthread cudnn ${CUDA_CUBLAS_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_curand_LIBRARY})
#add_dependencies(save_images ${PROJECT_NAME}_gencpp)
#add_dependencies(save_images ${PROJECT_NAME}_generate_messages_cpp)

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
# install(TARGETS apc_vision apc_vision
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_apc_vision.cpp)
# IF(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# ENDIF()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
