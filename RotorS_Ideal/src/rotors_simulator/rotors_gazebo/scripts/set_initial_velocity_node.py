#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist

def velocity_publisher_once(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
    rospy.init_node('velocity_publisher_once', anonymous=True)
    rospy.set_param('/use_sim_time', True)

    pub1 = rospy.Publisher('/pelican_1/velocity_topic', Twist, queue_size=10, latch=True)
    pub2 = rospy.Publisher('/pelican_2/velocity_topic', Twist, queue_size=10, latch=True)
    pub3 = rospy.Publisher('/pelican_3/velocity_topic', Twist, queue_size=10, latch=True)
    pub4 = rospy.Publisher('/pelican_4/velocity_topic', Twist, queue_size=10, latch=True)
    twist = Twist()
    twist.linear.x = float(linear_x)
    twist.linear.y = float(linear_y)
    twist.linear.z = float(linear_z)
    twist.angular.x = float(angular_x)
    twist.angular.y = float(angular_y)
    twist.angular.z = float(angular_z)

    pub1.publish(twist)
    pub2.publish(twist)
    pub3.publish(twist)
    pub4.publish(twist)
    rospy.loginfo("Published initial velocity message: {}".format(twist))

    # Loop until a subscriber is detected
    rate = rospy.Rate(1)  # Check every second
    while not rospy.is_shutdown():
        if pub1.get_num_connections() > 0 and pub2.get_num_connections() > 0 and pub3.get_num_connections() > 0 and pub4.get_num_connections() > 0:
            rospy.loginfo("Subscriber detected, shutting down.")
            rospy.signal_shutdown("Subscriber detected.")
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo("ROS shutdown request received.")
            break

if __name__ == '__main__':
    if len(sys.argv) == 7:
        velocity_publisher_once(*sys.argv[1:])
    else:
        print("Usage: velocity_publisher_once.py linear_x linear_y linear_z angular_x angular_y angular_z")
        sys.exit(1)

