#!/usr/bin/env python
import pymongo, rospy, sys, re, rosbag
from sensor_msgs.msg import Image, JointState, LaserScan
from nav_msgs.msg import Odometry

class BagConverter():
    """
    Transfer data from database to ROS bag file or from bag file to database.
    Search query should be given when retrieving data from database. Two samples of queries are 'author ServiceRobot' and 'tags lidar'.

    Parameters
    ----------
    server_ip , port, author, session_id, log_'SensorName', where_to_write and bag_name
    They are in config/server_config.yaml file, and README has details of these parameter.
    """
    def __init__(self):
        self.client = pymongo.MongoClient(rospy.get_param('server_ip'), rospy.get_param('port'))
        self.db = self.client.test_database

        rospy.init_node('mongo_bag_converter', anonymous=True)

    def get_data(self, query):
        """ get specific data from DB according to the given query """
        posts = self.db.posts.find(query)
        return posts

    def bag_to_db(self, filename):
        """ transfer data from bag file to database """
        bag = rosbag.Bag(filename)
        data_cnt = 0

        for topic,msg,t in bag.read_messages(topics=['/camera/rgb/image_raw']):
            post = {'session_id':rospy.get_param('session_id'),
                    'author':rospy.get_param('author'),
                    'time':t.to_sec(),
                    'tags':['image'],
                    'height':msg.height,
                    'width':msg.width,
                    'data':msg.data}
            post = self.db.posts
            post_id = posts.insert_one(post).inserted_id
            rospy.loginfo(post)
            data_cnt = data_cnt + 1

        for topic,msg,t in bag.read_messages(topics=['/joint_states']):
            post = {'session_id':rospy.get_param('session_id'),
                    'author':rospy.get_param('author'),
                    'time':t.to_sec(),
                    'tags':['joint_states'],
                    'joint_names':msg.name,
                    'position':msg.position,
                    'velocity':msg.velocity,
                    'effort':msg.effort}
            post = self.db.posts
            post_id = posts.insert_one(post).inserted_id
            rospy.loginfo(post)
            data_cnt = data_cnt + 1

        for topic,msg,t in bag.read_messages(topics=['/scan']):
            post = {'session_id':rospy.get_param('session_id'),
                    'author':rospy.get_param('author'),
                    'time':t.to_sec(),
                    'tags':['lidar'],
                    'time_increment':msg.time_increment,
                    'angle_min':msg.angle_min,
                    'angle_max':msg.angle_max,
                    'range_min':msg.range_min,
                    'range_max':msg.range_max,
                    'angle_increment':msg.angle_increment,
                    'ranges':msg.ranges}
            posts = self.db.posts
            post_id = posts.insert_one(post).inserted_id
            rospy.loginfo(post)
            data_cnt = data_cnt + 1

        for topic,msg,t in bag.read_messages(topics=['/odom']):
            post = {'session_id':rospy.get_param('session_id'),
                    'author':rospy.get_param('author'),
                    'time':t.to_sec(),
                    'tags':['odom'],
                    'position':[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                    'velocity':[msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]}
            posts = self.db.posts
            post_id = posts.insert_one(post).inserted_id
            rospy.loginfo(post)
            data_cnt = data_cnt + 1
        rospy.loginfo("Save bag to DB complete!")
        rospy.loginfo('number of data sent to DB: %d', data_cnt)

    def db_to_bag(self, query, filename):
        """ transfer data from database to bag file using query """
        bag = rosbag.Bag(filename, 'w')
        query_list = re.sub('[^\w]', ' ', query).split()
        rospy.loginfo(query_list)
        rospy.loginfo(query)
        field = query_list[0]
        value = query_list[1]
        query = {field:value}
        posts = self.get_data(query)
        try:
            rospy.loginfo('number of data in DB %s', posts.count())
            for post in posts:
                if post['tags'][0] == 'image':
                    msg = Image()
                    msg.height = post['height']
                    msg.width = post['width']
                    msg.data = post['data']
                    bag.write('/image_rgb', msg, t=rospy.Time.from_sec(post['time']))
                    rospy.loginfo(msg)

                if post['tags'][0]=='joint_states':
                    msg = JointState()
                    msg.name = post['joint_names']
                    msg.position = post['position']
                    msg.velocity = post['velocity']
                    msg.effort = post['effort']
                    bag.write('/JointState', msg, t=rospy.Time.from_sec(post['time']))
                    rospy.loginfo(msg)

                if post['tags'][0]=='lidar':
                    msg = LaserScan()
                    msg.time_increment = post['time_increment']
                    msg.angle_min = post['angle_min']
                    msg.angle_max = post['angle_max']
                    msg.range_min = post['range_min']
                    msg.range_max = post['range_max']
                    msg.angle_increment = post['angle_increment']
                    msg.ranges = post['ranges']
                    bag.write('/scan', msg, t=rospy.Time.from_sec(post['time']))
                    rospy.loginfo(msg)

                if post['tags'][0]=='odom':
                    msg = Odometry()
                    msg.pose.pose.position.x = post['position'][0]
                    msg.pose.pose.position.y = post['position'][1]
                    msg.pose.pose.position.z = post['position'][2]
                    msg.twist.twist.linear.x = post['velocity'][0]
                    msg.twist.twist.linear.y = post['velocity'][1]
                    msg.twist.twist.angular.z = post['velocity'][2]
                    bag.write('/odom', msg, t=rospy.Time.from_sec(post['time']))
                    rospy.loginfo(msg)
        finally:
            bag.close()
            rospy.loginfo("Save DB to bag complete!")
            rospy.loginfo('number of data read from DB %s', posts.count())


if __name__ == '__main__':
    bc = BagConverter()
    operation = rospy.get_param('where_to_write')
    if operation == 'write_database':
        rospy.loginfo("Store data into database")
        bc.bag_to_db(rospy.get_param('bag_path')+'/'+rospy.get_param('bag_name'))
    elif operation == 'write_bag':
        rospy.loginfo("Store data into bag file")
        bc.db_to_bag(rospy.get_param('write_query'), rospy.get_param('bag_path')+'/'+rospy.get_param('bag_name'))
    else:
        rospy.loginfo('NOT A VALID OPERATION!');
