cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# if not defined, set WEBOTS_HOME environment variable to default value
if (DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME /usr/local/webots)
endif()

##
## Build configuration for a Webots controller using the C++ API:
## 

# Webots includes
add_definitions(-I${WEBOTS_HOME}/include/controller/cpp -I${WEBOTS_HOME}/include -Wall -DLINUX)

# Put the built executable in the "controllers/joint_position_controller" directory to suit Webots,
# instead of the default "bin" directory for ROS
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/controllers/joint_position_controller)
rosbuild_add_executable(joint_position_controller src/webots_joint_position_controller.cpp)

# Link executable to Webots C++ Controller Library
find_library(CONTROLLER_LIBRARY CppController ${WEBOTS_HOME}/lib)
target_link_libraries(joint_position_controller ${CONTROLLER_LIBRARY})

##
## If you want to use only the Webots C Functions, you need to change the include
## path and library linking:
##
#add_definitions(-I${WEBOTS_HOME}/include/controller/c -I${WEBOTS_HOME}/include -Wall -DLINUX)
#find_library(CONTROLLER_LIBRARY Controller ${WEBOTS_HOME}/lib)

