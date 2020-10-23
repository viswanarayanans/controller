import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Odometry
from msg_check.msg import PlotDataMsg


def traj_callback(data):
	global first_msg, t
	ftraj.write("\nTrajectory:")
	ftraj.write("\nsecs:")
	ftraj.write(str(data.header.stamp.secs))
	ftraj.write("\nnsecs:")
	ftraj.write(str(data.header.stamp.nsecs))
	ftraj.write("\nx:")
	ftraj.write(str(data.points[0].transforms[0].translation.x))
	ftraj.write("\ny:")
	ftraj.write(str(data.points[0].transforms[0].translation.y))
	ftraj.write("\nz:")
	ftraj.write(str(data.points[0].transforms[0].translation.z))
	ftraj.write("\na_x:")
	ftraj.write(str(data.points[0].transforms[0].rotation.x))
	ftraj.write("\na_y:")
	ftraj.write(str(data.points[0].transforms[0].rotation.y))
	ftraj.write("\na_z:")
	ftraj.write(str(data.points[0].transforms[0].rotation.z))
	ftraj.write("\na_w:")
	ftraj.write(str(data.points[0].transforms[0].rotation.w))
	# ftraj.write("\ntime:")
	# if(first_msg == 1):
		# t1 = 10
		# t = time.time() - 10
	# 	first_msg = 0
	# else:
	# 	t1 = time.time() - t
	# ftraj.write(str(t1))
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
	fodom.write("\na_X:")
	fodom.write(str(data.pose.pose.orientation.x))
	fodom.write("\na_Y:")
	fodom.write(str(data.pose.pose.orientation.y))
	fodom.write("\na_Z:")
	fodom.write(str(data.pose.pose.orientation.z))
	fodom.write("\na_W:")
	fodom.write(str(data.pose.pose.orientation.w))


def error_callback(data):
	ferror.write("\nError:")
	ferror.write("\nsecs:")
	ferror.write(str(data.header.stamp.secs))
	ferror.write("\nnsecs:")
	ferror.write(str(data.header.stamp.nsecs))
	ferror.write("\nErr_X:")
	ferror.write(str(data.position_error.x))
	ferror.write("\nErr_Y:")
	ferror.write(str(data.position_error.y))
	ferror.write("\nErr_Z:")
	ferror.write(str(data.position_error.z))
	ferror.write("\nErr_phi:")
	ferror.write(str(data.angle_error.x))
	ferror.write("\nErr_theta:")
	ferror.write(str(data.angle_error.y))
	ferror.write("\nErr_psi:")
	ferror.write(str(data.angle_error.z))
	ferror.write("\nThrust:")
	ferror.write(str(data.thrust))
	ferror.write("\nMoments_x:")
	ferror.write(str(data.moments.x))
	ferror.write("\nMoments_y:")
	ferror.write(str(data.moments.y))
	ferror.write("\nMoments_z:")
	ferror.write(str(data.moments.z))


if __name__ == '__main__':
    rospy.init_node('odom_traj',anonymous=True)
    global first_msg, t
    first_msg = 1
    fodom = open("icra_2021/odom_elas_elong_wall.txt","w")
    ferror = open("icra_2021/error_elas_elong_wall.txt","w")
    ftraj = open("icra_2021/traj_elas_elong_wall.txt","w")
    t = time.time()

    # Subsciber
    rospy.Subscriber("/command/trajectory", MultiDOFJointTrajectory, traj_callback)
    rospy.Subscriber("/elasticopter/odometry_sensor1/odometry", Odometry, odom_callback)
    rospy.Subscriber("/data_out", PlotDataMsg, error_callback)

    # Publisher
    # pub = rospy.Publisher("/odometry/pose",PoseWithCovarianceStamped,queue_size=10)

    rospy.spin()
    