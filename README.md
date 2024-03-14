# Turtle_Mover


This C++ code defines a ROS node to control a turtle robot's movement. It subscribes to topics for receiving the current pose and goal position of the turtle. Upon receiving a goal, it calculates velocity commands to navigate the turtle towards the goal using proportional control. The main function initializes the ROS node, and the program continues to run until ROS exits, processing messages using callback functions for pose and goal updates.
