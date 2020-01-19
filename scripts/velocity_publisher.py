import rospy
import serial

# from apriltag_ros.msg import AprilTagDetectionArray
from mav_msgs.msg import Actuators
# from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    #print(data.angular_velocities)
    vel = data.angular_velocities
    vel_str = str(vel[0]) + ',' + str(vel[1]) + ',' + str(vel[2]) + ',' + str(vel[3]) + ',1\n'
    # print(vel_str)
    ser.write(vel_str)

if __name__ == '__main__':
    rospy.init_node('velocity_publisher',anonymous=True)

    # Subsciber
    rospy.Subscriber("/pelican/command/motor_speed", Actuators, callback)

    ser = serial.Serial('/dev/ttyUSB0', 115200)

    rospy.spin()
    