# Learning

## Policy-based Teleop Package
Incomplete work of Learning from Demonstration
We worked on learning from demonstration on the basis of inverse reinforcement learning.

In learning_from_demonstration package, there is teleop-based robot controller. The controller generates linear and angular velocity if the parameters of policy and pre-engineered features are given prior to the execution.

Details are in README of learning_from_demonstration.

## mongodb API
This work provides API to record data from various sources (e.g. image from kinect) to MongoDB database. Overall functions that this work provides are follows:
- Record data to database
- Publish data in database to ROS
- Transfer data in bag file to database
- Transfer data in database to bag file

More details can be found in README of mongodb_api package.
