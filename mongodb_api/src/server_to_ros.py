#!/usr/bin/env python

import pymongo, rospy, re
from mongodb_api.srv import *
from sensor_msgs.msg import Image, JointState, LaserScan
from nav_msgs.msg import Odometry

class MongoRetriever():
    """
    Retrieve data from database and publish them to ROS.
    Search query should be given when launch file for MongoRetriever is called. Two samples of queries are 'author ServiceRobot' and 'tags lidar'.

    Parameters
    ----------
    server_ip , port and data_rate
    They are in config/server_config.yaml file, and README has details of these parameter.
    """
    def __init__(self):
        self.client = pymongo.MongoClient(rospy.get_param('server_ip'), rospy.get_param('port'))
        self.db = self.client.test_database

        rospy.init_node('ros_mongo_retriever', anonymous=True)

        self.image_pub = rospy.Publisher('/camera/rgb/image_raw_mongo', Image, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states_mongo', JointState, queue_size=10)
        self.lidar_pub = rospy.Publisher('/scan_mongo', LaserScan, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom_mongo', Odometry, queue_size=10)

        rospy.loginfo('Initialized Mongo Retriever (server to ROS)')

    def get_data(self, query):
        """ collect specific data fit to query """
        posts = self.db.posts.find(query)
        return posts

    def pub_data(self, query):
        """ publish data from DB to ROS """
        rospy.loginfo('Start publishing data to ROS')
        posts = self.db.posts.find(query)
        rospy.loginfo('number of data in DB %s', posts.count())
        rate = rospy.Rate(rospy.get_param('data_rate'))
        for post in posts:
            if post['tags'][0]=='image':
                rospy.loginfo('publishing Image')
                msg = Image()
                msg.header.stamp = rospy.Time.from_sec(post['time'])
                msg.height = post['height']
                msg.width = post['width']
                msg.data = post['data']
                self.image_pub.publish(msg)
                rospy.loginfo(msg)

            if post['tags'][0]=='joint_state':
                rospy.loginfo('publishing Joint state')
                msg = JointState()
                msg.header.stamp = rospy.Time.from_sec(post['time'])
                msg.name = post['joint_names']
                msg.position = post['position']
                msg.velocity = post['velocity']
                msg.effort = post['effort']
                self.joint_pub.publish(msg)
                rospy.loginfo(msg)

            if post['tags'][0]=='lidar':
                rospy.loginfo('publishing Lidar')
                msg = LaserScan()
                msg.header.stamp = rospy.Time.from_sec(post['time'])
                msg.time_increment = post['time_increment']
                msg.angle_min = post['angle_min']
                msg.angle_max = post['angle_max']
                msg.range_min = post['range_min']
                msg.range_max = post['range_max']
                msg.angle_increment = post['angle_increment']
                msg.ranges = post['ranges']
                self.lidar_pub.publish(msg)
                rospy.loginfo(msg)


            if post['tags'][0]=='odom':
                rospy.loginfo('publishing Odometry')
                msg = Odometry()
                msg.header.stamp = rospy.Time.from_sec(post['time'])
                msg.pose.pose.position.x = post['position'][0]
                msg.pose.pose.position.y = post['position'][1]
                msg.pose.pose.position.z = post['position'][2]
                msg.twist.twist.linear.x = post['velocity'][0]
                msg.twist.twist.linear.y = post['velocity'][1]
                msg.twist.twist.angular.z = post['velocity'][2]
                self.odom_pub.publish(msg)
                rospy.loginfo(msg)
            rate.sleep()


if __name__ == '__main__':
    mr = MongoRetriever()
    
    query = rospy.get_param('search_query')

    query_list = re.sub('[^\w]', ' ', query).split()
    field, value = query_list[0], query_list[1]
    query = {field : value}

    rospy.loginfo(query)
    mr.pub_data(query)
