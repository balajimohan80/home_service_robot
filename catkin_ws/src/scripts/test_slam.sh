#!/bin/sh
cd $(pwd)/../..;
catkin_make

xterm -e "source devel/setup.bash;
	  export ROBOT_INITIAL_POSE='-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0';
          export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/map/maps.world"; 
	  roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 10

xterm -e "source devel/setup.bash;
	  roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5

xterm -e "source devel/setup.bash; 
	  roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

xterm -e "source devel/setup.bash; 
	  roslaunch turtlebot_teleop keyboard_teleop.launch "

