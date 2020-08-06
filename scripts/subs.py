#!/usr/bin/env python

import rospy

from viswa_control.msg import PlotDataMsg


def plot_callback(data):
	print(data)

if __name__ == '__main__':
	rospy.init_node('subs',anonymous=True)

	rospy.Subscriber("/data_out", PlotDataMsg, plot_callback)
	
	rospy.spin()