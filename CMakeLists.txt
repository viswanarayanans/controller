cmake_minimum_required(VERSION 2.8.3)
project(viswa_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES  pid_position_controller smc_position_controller asmc_position_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
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

# add_library(roll_pitch_yawrate_thrust_controller
#   src/library/roll_pitch_yawrate_thrust_controller.cpp
# )

add_library(smc_position_controller
  src/library/smc_position_controller.cpp
)

add_library(asmc_position_controller
  src/library/asmc_position_controller.cpp
)

target_link_libraries(pid_position_controller ${catkin_LIBRARIES})
add_dependencies(pid_position_controller ${catkin_EXPORTED_TARGETS})

# target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
# add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(smc_position_controller ${catkin_LIBRARIES})
add_dependencies(smc_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(asmc_position_controller ${catkin_LIBRARIES})
add_dependencies(asmc_position_controller ${catkin_EXPORTED_TARGETS})


add_executable(pid_position_controller_node src/nodes/pid_position_controller_node.cpp)
add_dependencies(pid_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_position_controller_node
  pid_position_controller ${catkin_LIBRARIES})

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

foreach(dir launch resource)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


install(TARGETS pid_position_controller smc_position_controller asmc_position_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pid_position_controller_node asmc_position_controller_node smc_position_controller_node waypoint_publisher hovering_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


