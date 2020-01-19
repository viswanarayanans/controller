import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Odometry



def traj_callback(data):
	global first_msg, t
	ftraj.write("\nTrajectory:")
	ftraj.write("\nx:")
	ftraj.write(str(data.points[0].transforms[0].translation.x))
	ftraj.write("\ny:")
	ftraj.write(str(data.points[0].transforms[0].translation.y))
	ftraj.write("\nz:")
	ftraj.write(str(data.points[0].transforms[0].translation.z))
	ftraj.write("\ntime:")
	if(first_msg == 1):
		t1 = 10
		t = time.time() - 10
		first_msg = 0
	else:
		t1 = time.time() - t
	ftraj.write(str(t1))
	ftraj.write("\n")
	

def odom_callback(data):
	fodom.write("\nOdometry:")
	fodom.write("\nsecs:")
	fodom.write(str(data.header.stamp.secs))
	fodom.write("\nnsecs:")
	fodom.write(str(data.header.stamp.nsecs))
	fodom.write("\nX:")
	fodom.write(str(data.pose.pose.position.x))
	fodom.write("\nY:")
	fodom.write(str(data.pose.pose.position.y))
	fodom.write("\nZ:")
	fodom.write(str(data.pose.pose.position.z))


if __name__ == '__main__':
    rospy.init_node('plot_sub',anonymous=True)
    global first_msg, t
    first_msg = 1
    fodom = open("odom_asmc.txt","w")
    ftraj = open("traj_asmc.txt","w")
    t = time.time()

    # Subsciber
    rospy.Subscriber("/pelican/command/trajectory", MultiDOFJointTrajectory, traj_callback)
    rospy.Subscriber("/pelican/odometry_sensor1/odometry", Odometry, odom_callback)

    # Publisher
    # pub = rospy.Publisher("/odometry/pose",PoseWithCovarianceStamped,queue_size=10)

    rospy.spin()
    