#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_forward():
    rospy.init_node('go_forward_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()
    twist.linear.x = 0.3   # Tiến về phía trước với tốc độ 0.3 m/s
    twist.angular.z = 0.0  # Không xoay

    rospy.loginfo("Xe dang chay thang...")

    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
