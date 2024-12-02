# Homework #4 Scorpion ReadMe

### Description of algorithm
We were tasked with developing a coverage based algorithm to navigate an unknown environment. Around the environment are april tags mounted on the wall. The goal of the assignment was to develop a controller that when given eight minutes to explore could find and identify as many tags as possible. Our algorithm is relatively simple. It instructs the robot to drive forward until it encounters a wall. It will then turn to follow the wall at a safe distance for as long as the controller runs. If the robot encounters an obstacle or gets to close to a wall, it will stop and breifly reverse before executing a turn to avoid the obstruction. In the open environment of the ring, this behavior makes it difficult for the robot to get stuck. In the background, and april tag detection node is ran so that the camera will pick up and detect any tags encountered. 

### How to run
Our launch file is name apriltag_h4_launch.py. It starts the controller, the camera node, and april tag node as required by the assignment. This can be ran by 
<pre>ros2 launch webots_ros2_homework1_python apriltag_h4_launch.py</pre>