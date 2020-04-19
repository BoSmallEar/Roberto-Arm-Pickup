cmake_minimum_required(VERSION 2.8.3)
project(roberto_ik_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  geometry_msgs
  std_msgs
  std_srvs
  moveit_msgs
  dynamic_reconfigure
  actionlib
  actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/SphericalGrasp.cfg
  )

add_action_files(
  FILES
  PickUpPose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs
)

catkin_package(
)

include_directories(
  SYSTEM
  scripts
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(plan_arm_torso_fk src/plan_arm_torso_fk.cpp)
add_dependencies(plan_arm_torso_fk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_arm_torso_fk
   ${catkin_LIBRARIES}
 )

add_executable(plan_arm_torso_ik src/plan_arm_torso_ik.cpp)
add_dependencies(plan_arm_torso_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_arm_torso_ik
   ${catkin_LIBRARIES}
 )


install(TARGETS plan_arm_torso_fk plan_arm_torso_ik
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  PROGRAMS
      scripts/ik_planner.py 
      scripts/listener.py 
      scripts/talker.py 
      scripts/pickup.py
      scripts/pick_and_place_server.py
      scripts/spherical_grasps_server.py
  DESTINATION 
      ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
