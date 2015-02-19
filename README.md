# rosban
ROS for SigmaBan

The ROS workspace is in: /home/rhoban/Project/ros_ws

Basic operation for Django:
- roslaunch rosban start_Django.launch
- roslaunch rosban manager_Django_imu.launch
- (optional, for the webcam) roslaunch rosban cam.launch

You now have access to the motors and the IMU.

NOTES:
- To compile: go to /home/rhoban/Project/ros_ws and type catkin_make (eventually catkin_make install)
- To run a node: rosrun PACKAGE_NAME NODE_NAME (ie: rosrun rosban test_motor)
- The custom ROS packages should be created in /home/rhoban/Project/ros_ws/src/
