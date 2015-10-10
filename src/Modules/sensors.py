from sensor_msgs.msg import LaserScan


rospy.Subscriber('/scan', LaserScan, somefunction, queue_size = 1)