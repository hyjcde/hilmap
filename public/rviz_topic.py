#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def rviz_demo_publisher():
    rospy.init_node('rviz_demo_publisher', anonymous=True)
    pub = rospy.Publisher('demo_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        rviz_demo_publisher()
    except rospy.ROSInterruptException:
        pass


