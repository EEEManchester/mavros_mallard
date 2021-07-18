#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn


# Receives joystick messages (subscribed to Joy topic)
# axis 1 aka left stick vertical
# axis 0 aka left stick horizonal
# axis 3 aka left stick horizonal

def callback(joystick):
 global pub
 # Joystick data input
 cmd_vel_y = float(joystick.axes[0])
 cmd_vel_x = float(joystick.axes[1])
 cmd_vel_r = float(joystick.axes[3])
 com_vel = OverrideRCIn()
 # Map the signal from [-1, 1] to [1100, 1900]
 cmd_vel_y = 1500 + (400 * cmd_vel_y)
 cmd_vel_x = 1500 + (400 * cmd_vel_x)
 cmd_vel_r = 1500 + (400 * cmd_vel_r)
 com_vel.channels = [0, 0, 0, cmd_vel_r, cmd_vel_x, cmd_vel_y, 0, 0]
 print com_vel
 pub.publish(com_vel)

def start():
 rospy.init_node('Joy2thr')
 global pub
 pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
 rospy.Subscriber("joy", Joy, callback)

if __name__ == '__main__':
 start()
 rospy.spin()
