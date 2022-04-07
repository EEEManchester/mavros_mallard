#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Receives joystick messages (subscribed to Joy topic)
# axis 1 aka left stick vertical
# axis 0 aka left stick horizontal

   
def joy2cmd_callback(joyData):
    global pub 
    # Receives joystick messages (subscribed to Joy topic)
    # axis 1 aka left stick vertical
    # axis 0 aka left stick horizonal
    cmd_vel_u = float(joyData.axes[1])
    cmd_vel_v = float(joyData.axes[0])
    cmd_vel_r = float(joyData.axes[3])
    cmd_ = Twist()
    cmd_.linear.x = cmd_vel_u
    cmd_.linear.y = cmd_vel_v
    cmd_.angular.z = cmd_vel_r
    #cmd_= np.matrix([[cmd_vel_u], [cmd_vel_v], [cmd_vel_r]])

    # print cmd_
    pub.publish(cmd_)



def joy2cmd():
    rospy.init_node('Joy2cmd')
    global pub
    pub = rospy.Publisher("mallard_cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, joy2cmd_callback)


if __name__ == '__main__':
    joy2cmd()
    rospy.spin()