# mongodb API

## Dependencies
Install [MongoDB](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/) and pymongo (pip install pymongo). Installation scripts are included in shell script install.sh.

Follow [instructions](https://www.howtoforge.com/tutorial/install-mongodb-on-ubuntu-16.04/) to set up MongoDB

## Set-up database
You may use mongo service directly or docker for mongoDB which is recommended for this course.
For the case of using docker, please refer to main README for installing/setting docker.
Description below assumes the name of docker is 700mongodb if using docker.

## Set-up parameters
Parameters for database are in config/server_config.yaml file.
Currently, there are 13 parameters, but it is possible to add more as types of data to record increase.
- server_ip : either 'localhost' (string) or ip address (e.g. 158.130.191.70 not as string)
- port : value for port (integer)
- data_rate : value for receiving/publishing data (Hz)
- log_sensor_name : True if recording data from this sensor, otherwise False
- author : put your name please (string)
- bag_name, where_to_write : only matters when transferring data between database and bag file (string)

## Launch files
A few sample launch files are provided in mongodb_api/launch directory.

- Launch server
    - Start MongoDB or docker in terminals.
        - `sudo service mongod start`
        - `docker start 700mongodb`
    - Start server of mongodb API using parameters in config/yaml file.
        - `roslaunch mongodb_api launch_server.launch`

- Publish sample ROS message
    - Start server of mongodb API using above commands.
    - Publish data using test_launch.launch.
        - `roslaunch mongodb_api test_launch.launch`

- Publish data in server as ROS message
    - Start server of mongodb API using above commands.
    - Retrieve data from the server using query as following format:
         - command format : `roslaunch mongodb_api retrieve.launch query:='field_name field_value'`
         - command example 1 : `roslaunch mongodb_api retrieve.launch query:='author ServiceRobot'`
         - command example 2 : `roslaunch mongodb_api retrieve.launch query:='tags lidar'`

- Save data from database to bag file
    - Start server of mongodb API using above commands.
    - Use bag_converter with appropriate parameter bag_name, where_to_write, and query. (Check config/yaml file, especially where_to_write : 'write_bag')
        - `roslaunch mongodb_api bag_converter.launch query:='tags lidar'`
    - Bag file will be saved in bag directory of mongodb_api.

- Save data from bag file to database
    - Start server of mongodb API using above commands.
    - Use bag_converter with appropriate parameter bag_name and where_to_write. (Check config/yaml file, especially where_to_write : 'write_database')
        - `roslaunch mongodb_api bag_converter.launch`
    - Launch file will search the bag file from bag directory of mongodb_api.

## Add/Edit sensor of this package
- Add/Edit config/yaml file to add new `log_sensor_name`.
- Edit src/mongo_server.py
    - Add new subscriber to MongoServer.__init__, or edit subscriber of the sensor.
    - Define new callback or edit existing callback function for the sensor in MongoServer.
- Edit src/server_to_ros.py
    - Add new subscriber to MongoRetriever.__init__, or edit subscriber of the sensor.
    - Make new block or edit existing block of generating ROS message for the sensor in method MongoRetriever.pub_data.
- Edit src/bag_converter.py
    - Make new block or edit existing block of sending data to DB in method BagConverter.bag_to_db.
    - Make new block or edit existing block of sending data to bag in method BagConverter.db_to_bag.

## Future work of this package
- Migrate from MongoDB to CouchDB
    - CouchDB has better GUI, and map_data_manager of navigation stack already made remote server connecting all service robots using CouchDB.
