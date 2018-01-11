#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

"""
Generate and publish fake LIDAR sensor's reading

Parameters
----------
data_rate
It is in config/server_config.yaml file, and README has details of this parameter.
"""

pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
rospy.init_node('test_pub_mongodb', anonymous=True)
rate = rospy.Rate(rospy.get_param('data_rate'))
max_message_cnt, pub_message_cnt = 200, 0

rospy.loginfo("Start publishing fake LaserScan messages to ROS")

while (not rospy.is_shutdown() and pub_message_cnt<max_message_cnt):
    message = LaserScan()
    message.angle_min = 0
    message.time_increment = 0.5
    message.angle_max = 100
    message.range_min = 10
    message.range_max = 20
    message.angle_increment = 20
    message.ranges = [10, 10, 20, 20, 10]
    pub.publish(message)
    pub_message_cnt = pub_message_cnt + 1
    rate.sleep()

rospy.loginfo("Complete publishing fake LaserScan messages")
