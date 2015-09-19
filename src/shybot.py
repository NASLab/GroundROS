#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def distanceArray(data):

    right_summation = 0
    left_summation = 0
    for dist in data.ranges[:271]:
        dist = min(dist, range_limit)
        right_summation = right_summation + dist

    for dist in data.ranges[271:]:
        dist = min(dist, range_limit)
        left_summation = left_summation + dist

    if right_summation > left_summation + margin:
        velocity = 1
    elif left_summation > right_summation + margin:
        velocity = -1
    else:
        velocity = 0

    twist_msg.angular.z = velocity
    pub.publish(twist_msg)


if __name__ == "__main__":
    machine = 'husky'
    try:
        twist_msg = Twist()
        margin = 5
        range_limit = 1.5

        rospy.init_node('shybot')
        message = "@%s: Node is being initiated" % (rospy.get_time())

        message = "@%s: Running readDistance Function" % (rospy.get_time())
        rospy.loginfo(message)

        pub = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1)

        message = "@%s: Running publisher" % (rospy.get_time())
        rospy.loginfo(message)
        rate = rospy.Rate(20)

        rospy.Subscriber('/scan', LaserScan, distanceArray)
        rospy.spin()

    except rospy.ROSInterruptException, e:
        print e
        message = "@%s: Feedback navigation terminated." % (rospy.get_time())
        rospy.loginfo(message)

    finally:
        message = "@%s: Node is shutting down." % (rospy.get_time())
        rospy.loginfo(message)

        pub.publish(Twist())
