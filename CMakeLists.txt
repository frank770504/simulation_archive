cmake_minimum_required(VERSION 2.8.3)
project(compal_gazebo)

find_package(catkin
  REQUIRED
    message_generation
    gazebo_ros
    urdf
)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_service_files(
  FILES
  SetModelPoseService.srv
  PushRodService.srv
)

generate_messages(
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    urdf
  DEPENDS
    gazebo
  LIBRARIES
    compal_gazebo_plugin
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(compal_gazebo_plugin src/plugin.cpp)
target_link_libraries(compal_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(compal_gazebo_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
install(
  TARGETS compal_gazebo_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
