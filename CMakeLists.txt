cmake_minimum_required(VERSION 2.8.3)
project(rrc_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  cmake_modules
  message_generation
  eigen_conversions
  serial_comm
  mav_trajectory_generation_ros
  msg_check
)

find_package(Eigen REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES  pid_position_controller smc_position_controller asmc_position_controller autde_position_controller reconfig_pid_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp rospy sensor_msgs
      message_runtime serial_comm mav_trajectory_generation_ros msg_check
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(pid_position_controller
  src/library/pid_position_controller.cpp
)

add_library(reconfig_pid_controller
  src/library/reconfig_pid_controller.cpp
)

# add_library(roll_pitch_yawrate_thrust_controller
#   src/library/roll_pitch_yawrate_thrust_controller.cpp
# )

add_library(smc_position_controller
  src/library/smc_position_controller.cpp
)

add_library(asmc_position_controller
  src/library/asmc_position_controller.cpp
)

add_library(autde_position_controller
  src/library/autde_position_controller.cpp
)

target_link_libraries(pid_position_controller ${catkin_LIBRARIES})
add_dependencies(pid_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(reconfig_pid_controller ${catkin_LIBRARIES})
add_dependencies(reconfig_pid_controller ${catkin_EXPORTED_TARGETS})

# target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
# add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(smc_position_controller ${catkin_LIBRARIES})
add_dependencies(smc_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(asmc_position_controller ${catkin_LIBRARIES})
add_dependencies(asmc_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(autde_position_controller ${catkin_LIBRARIES})
add_dependencies(autde_position_controller ${catkin_EXPORTED_TARGETS})


add_executable(pid_position_controller_node src/nodes/pid_position_controller_node.cpp)
add_dependencies(pid_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_position_controller_node
  pid_position_controller ${catkin_LIBRARIES})

add_executable(reconfig_pid_controller_node src/nodes/reconfig_pid_controller_node.cpp)
add_dependencies(reconfig_pid_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(reconfig_pid_controller_node
  reconfig_pid_controller ${catkin_LIBRARIES})

# add_executable(roll_pitch_yawrate_thrust_controller_node
#   src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
# add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(roll_pitch_yawrate_thrust_controller_node
#   roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})

add_executable(smc_position_controller_node src/nodes/smc_position_controller_node.cpp)
add_dependencies(smc_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(smc_position_controller_node
  smc_position_controller ${catkin_LIBRARIES})

add_executable(asmc_position_controller_node src/nodes/asmc_position_controller_node.cpp)
add_dependencies(asmc_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_position_controller_node
  asmc_position_controller ${catkin_LIBRARIES})

add_executable(autde_position_controller_node src/nodes/autde_position_controller_node.cpp)
add_dependencies(autde_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(autde_position_controller_node
  autde_position_controller ${catkin_LIBRARIES})


add_executable(waypoint_publisher src/nodes/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher ${catkin_EXPORTED_TARGETS})


# add_executable(pcl_pcl2 src/nodes/pcl_pcl2.cpp)
# target_link_libraries(pcl_pcl2 ${catkin_LIBRARIES})
# add_dependencies(pcl_pcl2 $(catkin_EXPORTED_TARGETS))

# add_executable(udp_publisher src/nodes/udp_publisher.cpp)
# target_link_libraries(udp_publisher ${catkin_LIBRARIES})
# add_dependencies(udp_publisher $(catkin_EXPORTED_TARGETS))


add_executable(hovering_example src/nodes/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})
add_dependencies(hovering_example ${catkin_EXPORTED_TARGETS})

add_executable(hovering_smooth src/nodes/hovering_smooth.cpp)
target_link_libraries(hovering_smooth ${catkin_LIBRARIES})
add_dependencies(hovering_smooth ${catkin_EXPORTED_TARGETS})

add_executable(points_publisher src/nodes/points_publisher.cpp)
target_link_libraries(points_publisher ${catkin_LIBRARIES})
add_dependencies(points_publisher ${catkin_EXPORTED_TARGETS})

foreach(dir launch resource)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


install(TARGETS pid_position_controller smc_position_controller asmc_position_controller autde_position_controller reconfig_pid_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pid_position_controller_node asmc_position_controller_node autde_position_controller_node
 smc_position_controller_node waypoint_publisher hovering_example hovering_smooth points_publisher reconfig_pid_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


