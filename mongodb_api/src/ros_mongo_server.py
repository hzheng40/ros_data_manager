#!/usr/bin/env python

import pymongo, rospy
from mongodb_api.srv import *
from sensor_msgs.msg import Image, JointState, LaserScan
from nav_msgs.msg import Odometry

class MongoServer():
    """
    Receive data from sources like sensors, and store them in database.
    New sensor can be added by making new subscriber for the sensor in __init__ and appropriate callback method.

    Parameters
    ----------
    server_ip , port, author, session_id and log_'SensorName'
    They are in config/server_config.yaml file, and README has details of these parameter.
    """
    def __init__(self):
        self.client = pymongo.MongoClient(rospy.get_param('server_ip'), rospy.get_param('port'))
        self.db = self.client.test_database

        rospy.init_node('ros_mongo_server', anonymous=True)

        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1, buff_size=2 ** 24)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("mongodb_api Server Initialized")
        rospy.spin()

    def image_callback(self, data):
        """ handle Image message published from kinect """
        if rospy.get_param('log_image'):
            post = {"session_id":rospy.get_param('session_id'),
                    "author":rospy.get_param('author'), 
                    "time":rospy.get_time(), 
                    "tags":["image"], 
                    "height":data.height, 
                    "width":data.width, 
                    "data":data.data}
            posts = self.db.posts
            post_id = posts.insert_one(post).inserted_id

    def joint_callback(self, data):
        """ handle JointState message published from robot base and possibly robot arm """
        if rospy.get_param('log_joint'):
            post = {"session_id":rospy.get_param('session_id'),
                    "author":rospy.get_param('author'), 
                    "time":rospy.get_time(), 
                    "tags":["joint_states"], 
                    "joint_names":data.name, 
                    "position":data.position, 
                    "velocity":data.velocity, 
                    "effort":data.effort}
            posts = self.db.posts
            post_id = posts.insert_one(post).inserted_id

    def lidar_callback(self, data):
        """ handle LaserScan message published from Lidar """
        if rospy.get_param('log_lidar'):
            post = {"session_id":rospy.get_param('session_id'),
                    "author": rospy.get_param('author'), 
                    "time":rospy.get_time(), 
                    "tags":["lidar"], 
                    "time_increment": data.time_increment,
                    "angle_min":data.angle_min, 
                    "angle_max": data.angle_max, 
                    "range_min": data.range_min,
                    "range_max": data.range_max,
                    "angle_increment":data.angle_increment, 
                    "ranges":data.ranges}
            posts = self.db.posts
            post_id = posts.insert_one(post).inserted_id

    def odom_callback(self, data):
        """ handle Odometry message published from odometry """
        if rospy.get_param('log_odom'):
            post = {"session_id":rospy.get_param('session_id'),
                    "author":rospy.get_param('author'), 
                    "time":rospy.get_time(), 
                    "tags":["odom"], 
                    "position":[data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z], 
                    "velocity":[data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z]}
            posts = self.db.posts
            post_id = posts.insert_one(post).inserted_id

if __name__ == "__main__":
    ms = MongoServer()
