cmake_minimum_required(VERSION 2.8.3)
project(rrc_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  mav_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  cmake_modules
  message_generation
  eigen_conversions
  mav_trajectory_generation_ros
  dynamic_reconfigure
)

find_package(Eigen REQUIRED)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PlotDataMsg.msg
   BoxMsg.msg
   PosVelMsg.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# generate_dynamic_reconfigure_options(
#   cfg/GeometricController.cfg
# )

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES  pid_position_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp rospy sensor_msgs
      message_runtime mav_trajectory_generation_ros std_msgs
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

add_library(smc_position_controller
  src/library/smc_position_controller.cpp
)

add_library(elas_smc_position_controller
  src/library/elas_smc_position_controller.cpp
)

add_library(asmc_position_controller
  src/library/asmc_position_controller.cpp
)

add_library(autde_position_controller
  src/library/autde_position_controller.cpp
)

add_library(sb_position_controller
  src/library/sb_position_controller.cpp
)

add_library(rsb_position_controller
  src/library/rsb_position_controller.cpp
)

add_library(rsbv_position_controller
  src/library/rsbv_position_controller.cpp
)

add_library(sac_position_controller
  src/library/sac_position_controller.cpp
)

add_library(asmc_mavros_controller
  src/library/asmc_mavros_controller.cpp
)

add_library(math
  src/library/math.cpp
)

target_link_libraries(pid_position_controller ${catkin_LIBRARIES})
add_dependencies(pid_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(reconfig_pid_controller ${catkin_LIBRARIES})
add_dependencies(reconfig_pid_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
# add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(smc_position_controller ${catkin_LIBRARIES})
add_dependencies(smc_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(elas_smc_position_controller ${catkin_LIBRARIES})
add_dependencies(elas_smc_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(asmc_position_controller ${catkin_LIBRARIES})
add_dependencies(asmc_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(autde_position_controller ${catkin_LIBRARIES})
add_dependencies(autde_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(sb_position_controller ${catkin_LIBRARIES})
add_dependencies(sb_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(rsb_position_controller ${catkin_LIBRARIES})
add_dependencies(rsb_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(rsbv_position_controller ${catkin_LIBRARIES})
add_dependencies(rsbv_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(sac_position_controller ${catkin_LIBRARIES})
add_dependencies(sac_position_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(asmc_mavros_controller ${catkin_LIBRARIES})
add_dependencies(asmc_mavros_controller rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(math ${catkin_LIBRARIES})
add_dependencies(math ${catkin_EXPORTED_TARGETS})


add_executable(pid_position_controller_node src/nodes/pid_position_controller_node.cpp)
add_dependencies(pid_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_position_controller_node
  pid_position_controller ${catkin_LIBRARIES})

add_executable(reconfig_pid_controller_node src/nodes/reconfig_pid_controller_node.cpp)
add_dependencies(reconfig_pid_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(reconfig_pid_controller_node
  reconfig_pid_controller ${catkin_LIBRARIES})


add_executable(reconfig_pid_controller_node_sim src/nodes/reconfig_pid_controller_node_sim.cpp)
add_dependencies(reconfig_pid_controller_node_sim rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(reconfig_pid_controller_node_sim
  reconfig_pid_controller ${catkin_LIBRARIES})

# add_executable(roll_pitch_yawrate_thrust_controller_node
#   src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
# add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(roll_pitch_yawrate_thrust_controller_node
#   roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})

add_executable(smc_position_controller_node src/nodes/smc_position_controller_node.cpp)
add_dependencies(smc_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(smc_position_controller_node
  smc_position_controller ${catkin_LIBRARIES})

add_executable(elas_smc_position_controller_node src/nodes/elas_smc_position_controller_node.cpp)
add_dependencies(elas_smc_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(elas_smc_position_controller_node
  elas_smc_position_controller ${catkin_LIBRARIES})

add_executable(asmc_position_controller_node src/nodes/asmc_position_controller_node.cpp)
add_dependencies(asmc_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_position_controller_node
  asmc_position_controller ${catkin_LIBRARIES})

add_executable(autde_position_controller_node src/nodes/autde_position_controller_node.cpp)
add_dependencies(autde_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(autde_position_controller_node
  autde_position_controller ${catkin_LIBRARIES})

add_executable(sb_position_controller_node src/nodes/sb_position_controller_node.cpp)
add_dependencies(sb_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(sb_position_controller_node
  sb_position_controller ${catkin_LIBRARIES})

add_executable(rsb_position_controller_node src/nodes/rsb_position_controller_node.cpp)
add_dependencies(rsb_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(rsb_position_controller_node
  rsb_position_controller ${catkin_LIBRARIES})

add_executable(rsbv_position_controller_node src/nodes/rsbv_position_controller_node.cpp)
add_dependencies(rsbv_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(rsbv_position_controller_node
  rsbv_position_controller ${catkin_LIBRARIES})

add_executable(sac_position_controller_node src/nodes/sac_position_controller_node.cpp)
add_dependencies(sac_position_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(sac_position_controller_node
  sac_position_controller ${catkin_LIBRARIES})

add_executable(asmc_mavros_controller_node src/nodes/asmc_mavros_controller_node.cpp)
add_dependencies(asmc_mavros_controller_node rrc_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_mavros_controller_node
  asmc_mavros_controller ${catkin_LIBRARIES})


add_executable(points_publisher src/nodes/points_publisher.cpp)
target_link_libraries(points_publisher ${catkin_LIBRARIES})
add_dependencies(points_publisher ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_publisher src/nodes/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher ${catkin_EXPORTED_TARGETS})

# add_executable(controller_node src/nodes/controller_node.cpp)
# target_link_libraries(controller_node ${catkin_LIBRARIES})
# add_dependencies(controller_node ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example src/nodes/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})
add_dependencies(hovering_example ${catkin_EXPORTED_TARGETS})

add_executable(pickup_drop src/nodes/pickup_drop.cpp)
target_link_libraries(pickup_drop ${catkin_LIBRARIES})
add_dependencies(pickup_drop ${catkin_EXPORTED_TARGETS})

add_executable(traj_gen src/nodes/traj_gen.cpp)
target_link_libraries(traj_gen ${catkin_LIBRARIES})
add_dependencies(traj_gen ${catkin_EXPORTED_TARGETS})

add_executable(ring_traj src/nodes/ring_traj.cpp)
target_link_libraries(ring_traj ${catkin_LIBRARIES})
add_dependencies(ring_traj ${catkin_EXPORTED_TARGETS})

add_executable(lyap_tester src/nodes/lyap_tester.cpp)
target_link_libraries(lyap_tester math ${catkin_LIBRARIES})
add_dependencies(lyap_tester ${catkin_EXPORTED_TARGETS})

add_executable(init_position src/nodes/init_position.cpp)
target_link_libraries(init_position ${catkin_LIBRARIES})
add_dependencies(init_position ${catkin_EXPORTED_TARGETS})

add_executable(hovering_smooth src/nodes/hovering_smooth.cpp)
target_link_libraries(hovering_smooth ${catkin_LIBRARIES})
add_dependencies(hovering_smooth ${catkin_EXPORTED_TARGETS})

add_executable(offboard_att src/nodes/offboard_att.cpp)
target_link_libraries(offboard_att ${catkin_LIBRARIES})
add_dependencies(offboard_att ${catkin_EXPORTED_TARGETS})

add_executable(up_and_down src/nodes/up_and_down.cpp)
target_link_libraries(up_and_down ${catkin_LIBRARIES})
add_dependencies(up_and_down ${catkin_EXPORTED_TARGETS})

add_executable(circle src/nodes/circle.cpp)
target_link_libraries(circle ${catkin_LIBRARIES})
add_dependencies(circle ${catkin_EXPORTED_TARGETS})

foreach(dir launch resource)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


install(TARGETS pid_position_controller math reconfig_pid_controller autde_position_controller asmc_mavros_controller smc_position_controller elas_smc_position_controller
  reconfig_pid_controller sac_position_controller asmc_mavros_controller sb_position_controller rsb_position_controller rsbv_position_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pid_position_controller_node reconfig_pid_controller_node reconfig_pid_controller_node_sim smc_position_controller_node asmc_mavros_controller_node 
  asmc_position_controller_node elas_smc_position_controller_node autde_position_controller_node sac_position_controller_node sb_position_controller_node 
  rsb_position_controller_node rsbv_position_controller_node points_publisher waypoint_publisher hovering_example pickup_drop traj_gen ring_traj lyap_tester
  init_position hovering_smooth offboard_att up_and_down circle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


