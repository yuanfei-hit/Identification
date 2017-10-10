cmake_minimum_required(VERSION 2.8.3)
project(lwr_sdh_planner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    trac_ik_lib
    roscpp
    geometry_msgs
    sensor_msgs
    trajectory_msgs    
    exotica
    ik_solver
    task_map
    moveit_msgs
    identification_msgs
)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
    roscpp
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    exotica
    ik_solver
    task_map
    moveit_msgs
    identification_msgs
  DEPENDS
    Boost
    orocos_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(planner src/planner.cpp)
target_link_libraries(planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
add_dependencies(planner identification_msgs_gencpp)


install(TARGETS planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(lwr_end_pos_pub_for_test src/lwr_end_pos_pub_for_test.cpp)
target_link_libraries(lwr_end_pos_pub_for_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)


install(TARGETS lwr_end_pos_pub_for_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
