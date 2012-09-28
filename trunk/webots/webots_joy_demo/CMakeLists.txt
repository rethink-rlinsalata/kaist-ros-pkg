cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()




if (DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    #set(WEBOTS_HOME $ENV{WEBOTS_HOME})
    set(WEBOTS_HOME /usr/local/webots)
endif()


# Webots includes
add_definitions(-I${WEBOTS_HOME}/include/controller/cpp -I${WEBOTS_HOME}/include -Wall -I${WEBOTS_HOME}/include/controller/c -DLINUX)

# Set default path for built executable to the "controllers/joystick" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/controllers/joystick)
rosbuild_add_executable(joystick src/joystick.cpp)

# Link executable to Webots Controller Library
find_library(CONTROLLER_LIBRARY Controller ${WEBOTS_HOME}/lib)
target_link_libraries(joystick ${CONTROLLER_LIBRARY})

