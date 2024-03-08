#!/usr/bin/env python

import rospy
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

from geometry_msgs.msg import Vector3

from nav_msgs.msg import Odometry
import tf
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16, Int32, Int64

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("PiHeartbeatNode")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######


        self.rate = rospy.get_param('~rate',1.0)  # the rate at which to publish the transform
        
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.then = rospy.Time.now()
        self.hb = 0
        self.Pub = rospy.Publisher("pi_heartbeat", Int64,queue_size=10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        
        self.hb += 1
        self.Pub.publish(self.hb)
            

    #############################################################################


#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.spin()
    
    
   
