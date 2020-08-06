import rospy

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
	global first_msg
	print(first_msg)

	try:
		odom = data.detections[0].pose # first detected tag
    	print(odom)

        # Publish
        pub.publish(odom)
	except IndexError:
		print("Tag not detected")

if __name__ == '__main__':
	first_msg = 0
	rospy.init_node('tag_to_odom',anonymous=True)

    # Subsciber
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)

    # Publisher
	pub = rospy.Publisher("/odometry/pose",PoseWithCovarianceStamped,queue_size=10)
	callback(5)

	rospy.spin()
    