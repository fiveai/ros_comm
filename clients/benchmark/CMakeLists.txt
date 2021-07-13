cmake_minimum_required(VERSION 2.8.3)
project(benchmark)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES benchmark
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_util
    STATIC
    src/Util.cpp
)


add_executable(${PROJECT_NAME}_publisher src/Publisher.cpp)
add_executable(${PROJECT_NAME}_subscriber src/Subscriber.cpp)
add_executable(${PROJECT_NAME}_monitor src/Monitor.cpp)

#target_link_libraries(${CMAKE_THREAD_LIBS_INIT})(${PROJECT_NAME}_publisher rt)
add_custom_target(${PROJECT_NAME}_all)
add_dependencies(${PROJECT_NAME}_all
    ${PROJECT_NAME}_publisher
    ${PROJECT_NAME}_subscriber
    ${PROJECT_NAME}_monitor
)

target_link_libraries(${PROJECT_NAME}_publisher
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_util
    rt
)

target_link_libraries(${PROJECT_NAME}_subscriber
    ${PROJECT_NAME}_util
    ${catkin_LIBRARIES}
    rt
)

target_link_libraries(${PROJECT_NAME}_monitor
    ${PROJECT_NAME}_util
    ${catkin_LIBRARIES}
)

install(
  TARGETS 
    ${PROJECT_NAME}_publisher
    ${PROJECT_NAME}_subscriber
    ${PROJECT_NAME}_monitor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 install(FILES
    launch/launch.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
