cmake_minimum_required(VERSION 3.10)
project(robot_gazebo_sim)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        realtime_tools
        gazebo_dev
        gazebo_ros
        urdf
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        robot_gazebo_sim_plugin
        CATKIN_DEPENDS
        roscpp
        sensor_msgs
        std_msgs
        realtime_tools
        gazebo_ros
        urdf
        DEPENDS
        )

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(robot_gazebo_sim_plugin
        src/robot_gazebo_sim.cpp
        )

add_dependencies(robot_gazebo_sim_plugin
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(robot_gazebo_sim_plugin
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

## Declare a cpp executable
add_executable(joint_control
        src/joint_control.cpp
        )

add_dependencies(joint_control
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(joint_control
        ${catkin_LIBRARIES}
        )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target robot_gazebo_sim_plugin")
    add_clang_tooling(
            TARGETS robot_gazebo_sim_plugin
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS robot_gazebo_sim_plugin joint_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(FILES robot_gazebo_sim_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch"
        )

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.world"
        )

