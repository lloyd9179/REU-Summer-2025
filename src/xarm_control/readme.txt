For controlling the xarm through this package:

First run the setup launch file which initializes the required nodes from the xarm_ros2 package
To do this run the following command
	ros2 launch xarm_control xarm_setup.launch.py
This will bring up the rviz as well as setup the services that will be called

After this run the following command
	ros2 launch xarm_control xarm_control.launch.py
This will run initialize the node that calls the services from the planner and executes motion plans
Currently it only goes to an initial pose however by setting up a subscriber, it can do the subsequently
request position looping
