#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
   
base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
def main():
    rospy.init_node('el movimiento de antonio')
    rospy.loginfo('el movimiento de antonio')
    # Subscriber for joint states
    sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
   
    rospy.spin()

def process_callback(msg):
    mensaje=Twist2DStamped()
    if msg.axes[0] > 0.1:
        mensaje.header.stamp = rospy.get_rostime()
        mensaje.omega = 1
        base_pub.publish(mensaje)
    if msg.axes[0] < -0.1:
        mensaje.header.stamp = rospy.get_rostime()
        mensaje.omega = -1
        base_pub.publish(mensaje)
    if msg.axes[1] > 0.1:
        mensaje.header.stamp = rospy.get_rostime()
        mensaje.v = 1
    if msg.axes[1] < 0.1
        
    rospy.loginfo(msg)
    rospy.loginfo(msg.axes[0])

if __name__ == '__main__':
    main()

