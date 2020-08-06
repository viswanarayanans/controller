import rospy
import serial
import numpy as np

# from apriltag_ros.msg import AprilTagDetectionArray
from mav_msgs.msg import Actuators
# from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    global W
    #print(data.angular_velocities)
    vel = 6*np.array(data.angular_velocities)
    R = np.array([np.ones(len(vel)), vel, vel**2]).T

    vel_str = str(W[0].dot(R[0])) + ',' + str(W[1].dot(R[1])) + ',' + str(W[2].dot(R[2])) + ',' + str(W[3].dot(R[3])) + ',1\n'
    print(vel_str)
    # ser.write(vel_str)

if __name__ == '__main__':
    W = np.array([[918.96891, 0.048367, 7.26316e-6],
        [905.01605, 0.050611, 7.045246e-6],
        [885.098643, 0.060828, 6.130089e-6],
        [1123.49376, 0.00845071, 9.486336e-6]])
    rospy.init_node('velocity_publisher',anonymous=True)

    # Subsciber
    rospy.Subscriber("/pelican/command/motor_speed", Actuators, callback)

    # ser = serial.Serial('/dev/ttyUSB0', 115200)

    rospy.spin()
    