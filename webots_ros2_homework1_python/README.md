# Final Project


### How to run the controller

1. Set up environment variables for ROS. 
<pre>
source /opt/ros/humble/setup.bash
</pre>
Also do any Windows or Mac specific setup

For example in Mac...
<pre>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
</pre>

For example in windows...
<pre>
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
</pre>

2. Build and Source the Package
<pre>
cd webots_ros2_homework1_python
colcon build
source install/setup.bash
</pre>

3. Launch the Webots Environment
Run this command and replace "name of world file" with the name of the maze you want to run or omit the argument entirely and it will default to maze.wbt
<pre>
ros2 launch webots_ros2_homework1_python f23_robotics_1_launch.py world:="name of world file"
</pre>

4. Launch the Controller
In a separate terminal source ros2 again and source the install (no need to rebuild)
<pre>
ros2 launch webots_ros2_homework1_python controller_launch.py
</pre>




