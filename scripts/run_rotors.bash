rosrun rotors_gazebo waypoint_publisher 0 0 3 0 __ns:=pelican
sleep 10
rosrun rotors_gazebo waypoint_publisher 0 2 2 0 __ns:=pelican
sleep 10
rosrun rotors_gazebo waypoint_publisher 2 0 2 90 __ns:=pelican
sleep 10
rosrun rotors_gazebo waypoint_publisher 2 2 4 0 __ns:=pelican
sleep 10
rosrun rotors_gazebo waypoint_publisher 2 2 2 0 __ns:=pelican
sleep 2