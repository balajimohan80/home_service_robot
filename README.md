# Home Service Robot

Final Project for Robotics Software Engineer

**Pick_Up_Zone**
<p align="center"><img src="./image/Pick_up_zone.gif"></p>

**Drop_Off_Zone**
<p align="center"><img src="./image/Drop_off_zone.gif"></p>

**Project Definition**

The robot must navigate desired pick-up zone to pick up the objects and deliver them to the drop-off zone. To navigate from source to destination, the robot first computes the shortest route path planning from source to destination. When the robot is traveling to its destination, it must constantly monitor its atmospheric surroundings and the robot has to reach its destination without hitting/crashing on obstacles.

In this project, the following software simulator and packages are used:
1.	Gazebo - Robot simulation tool.
2.	ROS package – Kinetic
3.	Platform OS – Ubuntu
4.	Software languages Used – Python, C++
5.	Robot software package – TurtleBot (https://www.turtlebot.com/)

Following git-hub ROS packages are used:
1.	https://github.com/ros-perception/slam_gmapping
2.	https://github.com/turtlebot/turtlebot
3.	https://github.com/turtlebot/turtlebot_interactions
4.	https://github.com/turtlebot/turtlebot_simulator

### Directory Layout

catkin_ws
└── src
    ├── add_markers
    |   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src
    │       ├── add_markers_node.cpp
    │       └── add_markers_node_test.cpp
    ├── map
    │   ├── maps.pgm
    │   ├── maps.world
    │   └── maps.yaml
    ├── pick_objects
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src
    │       ├── pick_objects_node.cpp
    │       └── pick_objects_node_orig.cpp
    ├── scripts
    │   ├── add_markers.sh
    │   ├── home_service.sh
    │   ├── pick_objects.sh
    │   ├── test_navigation.sh
    │   └── test_slam.sh
    ├── slam_gmapping
    │   ├── README.md
    │   ├── gmapping
    │   └── slam_gmapping
    ├── turtlebot
    │   ├── LICENSE
    │   ├── README.md
    │   ├── setup_create.sh
    │   ├── setup_kobuki.sh
    │   ├── turtlebot
    │   ├── turtlebot.rosinstall
    │   ├── turtlebot_bringup
    │   ├── turtlebot_capabilities
    │   ├── turtlebot_description
    │   └── turtlebot_teleop
    ├── turtlebot_interactions
    │   ├── README.md
    │   ├── turtlebot_dashboard
    │   ├── turtlebot_interactions
    │   ├── turtlebot_interactive_markers
    │   └── turtlebot_rviz_launchers
    └── turtlebot_simulator
        ├── README.md
        ├── turtlebot_gazebo
        ├── turtlebot_simulator
        ├── turtlebot_simulator.rosinstall
        ├── turtlebot_stage
        └── turtlebot_stdr
93 directories, 284 files



