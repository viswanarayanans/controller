# import RPi.GPIO as GPIO
import numpy as np
import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32

# from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_from_euler, euler_from_matrix
from mav_msgs.msg import Actuators


def odom_callback(data):
    global initial_position, initial_angles
    try:
		odom = data.detections[0].pose # first detected tag
		odom_new = PoseWithCovarianceStamped()
	        #print(np.array([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]))
		#trans = np.eye(4)
		#trans = quaternion_matrix(np.array([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
		#	odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]))
		#print(trans)
		#trans[0,3] = odom.pose.pose.position.x
		#trans[1,3] = odom.pose.pose.position.y
		#trans[2,3] = odom.pose.pose.position.z
		#print(trans[:,3])
		#rot = np.eye(4)
		#rot = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
		#trans_inv = rot.dot(np.linalg.inv(trans))
		#print("Location")
		#pos = trans_inv.dot(np.array([0, 0, 0, 1]))
		#print(pos)

		#print(euler_from_matrix(trans_inv, 'rxyz'))

		#trans = np.linalg.inv(trans)
		#trans[:3, :3] = rot.dot(trans[:3, :3])

		#print(trans)

		#print(rot)
		#print("")

		angles = euler_from_quaternion(np.array([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
				odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]))
		#print(np.array(angles))
		angles_reset = np.array([-angles[2]+initial_angles[2] , angles[0]-initial_angles[0], \
				angles[1]-initial_angles[1]])
		#print(angles_reset)

		quat = quaternion_from_euler(angles_reset[0], angles_reset[1], angles_reset[2])
		#print(quat)

		odom_new.pose.pose.orientation.x = quat[0]
		odom_new.pose.pose.orientation.y = quat[1]
		odom_new.pose.pose.orientation.z = quat[2]
		odom_new.pose.pose.orientation.w = quat[3]


		#print(np.linalg.inv(rot))
		#print("\n")

		odom_new.header = odom.header
		odom_new.pose.pose.position.x = -odom.pose.pose.position.z + initial_position[2]
		#print(-odom.pose.pose.position.z)
		#print("")
		odom_new.pose.pose.position.y = odom.pose.pose.position.x - initial_position[0]
		#print("")
		odom_new.pose.pose.position.z = odom.pose.pose.position.y - initial_position[1]
		#print(-odom.pose.pose.position.y)
		#print(odom_new)
		#print("")

	    # Publish
		pub.publish(odom_new)
    except IndexError:
        print("Tag not detected")


def mag_callback(data):
	print(data.data)
	print(len(data.data))
	if data.data == 'ON':
		print("Switching ON the magnet")
		GPIO.output(15, GPIO.HIGH)
	elif data.data == "OFF":
		print("Switching OFF the magnet")
		GPIO.output(15, GPIO.LOW)
	else:
		print("Invalid format")


def kill_callback(data):
	global kill
	print("Killing robots")
	kill = 0


def initialize(filename):
    global initial_position, orientation
    with open(filename,) as f:
        for i in f.readlines():
            k = i.split(':')
            if k[0] == 'Position':
                initial_position = np.fromstring(k[1].rstrip(), dtype=float, sep=',')
            if k[0] == 'Orientation':
                orientation = np.fromstring(k[1].rstrip(), dtype=float, sep=',')

def rpm_callback(data):
    global W
    #print(data.angular_velocities)
    vel = kill*np.array(data.angular_velocities)
    R = np.array([np.ones(len(vel)), vel, vel**2]).T

    vel_str = str(int(W[0].dot(R[0]))) + ',' + str(int(W[1].dot(R[1]))) + ',' + str(int(W[2].dot(R[2]))) + ',' + str(int(W[3].dot(R[3]))) + ',1\n'
    print(vel_str)
    # ser.write(vel_str)

if __name__ == '__main__':
    rospy.init_node('tag_to_odom',anonymous=True)
    # GPIO.setmode(GPIO.BOARD)
    # GPIO.setup(15, GPIO.OUT)

    kill = 1
    initial_position = np.zeros(3)
    orientation = np.zeros(4)
    # W = np.array([[918.96891, 0.048367, 7.26316e-6],
    #     [905.01605, 0.050611, 7.045246e-6],
    #     [885.098643, 0.060828, 6.130089e-6],
    #     [1123.49376, 0.00845071, 9.486336e-6]])

    W = np.array([[918.96891, 0.048367, 7.26316e-6],
        [918.96891, 0.048367, 7.26316e-6],
        [918.96891, 0.048367, 7.26316e-6],
        [918.96891, 0.048367, 7.26316e-6]])

    # initialize('init_pose.txt')
    print("Initial Position:")
    print(initial_position, type(initial_position))
    print(len(initial_position))
    print("Initial Orientation")
    print(orientation, type(orientation), len(orientation))
    initial_angles = euler_from_quaternion(orientation)

    # Subscibers
    # rospy.Subscriber("/tag_detections", AprilTagDetectionArray, odom_callback)
    rospy.Subscriber("/magnet_control", String, mag_callback)
    rospy.Subscriber("/pelican/command/motor_speed", Actuators, rpm_callback)
    rospy.Subscriber("/kill_all", Float32, kill_callback)


    # Publisher
    pub = rospy.Publisher("/odometry/pose", PoseWithCovarianceStamped, queue_size=10)

    rospy.spin()
