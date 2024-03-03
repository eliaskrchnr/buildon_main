#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


def callback(data):
    x = data.linear.x
    z = data.angular.z
    rospy.loginfo("Received Twist and converted to: Linear: %s" % x + " and Angular: %s" % z)



    
    pub_linear.publish(x)
    pub_angular.publish(z)


if __name__ == '__main__':
    rospy.init_node('convert_twist_to_int', anonymous=True)
    pub_linear = rospy.Publisher('/cmd_vel_linear', Float32, queue_size=10)
    pub_angular = rospy.Publisher('/cmd_vel_angular', Float32, queue_size=10)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.loginfo("started node")
    rospy.spin()