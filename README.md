# Mapping + exploration for TurtleBot using ROS2
Demo: [YouTube](https://www.youtube.com/watch?v=RguxtF7eiqg).
## How to run

Source the ROS2 installation. Will vary depending on your system.
```bash
# ROS2 setup source
$ source /opt/ros/humble/setup.zsh
$ export ROS_DOMAIN_ID=42
$ export ROS_LOCALHOST_ONLY=1
$ source /usr/share/colcon_cd/function/colcon_cd.sh
$ export _colcon_cd_root=/opt/ros/humble/
$ source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

Build the nodes. Should be done from a ROS2 workspace.
```bash
# assuming you are in the turtlebot-additions directory:
$ cd ../.. && colcon build 
# or from a ROS2 workspace:
$ colcon build
```

Source the built nodes:
```bash
$ source install/setup.bash
```

Prepare the TurtleBot source:
```bash
$ export TURTLEBOT3_MODEL=burger
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix my_turtlebot)/share/my_turtlebot/models
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
```

Launch the TurtleBot simulation
```bash
$ ros2 launch my_turtlebot turtlebot_simulation.launch.py
```

In the ```rviz``` window, create a 2D Pose estimation on top of the robot. Then, start the mapping node:

```bash
$ ros2 run turtle_localization mapping
```

Start the frontier node:

```bash
$ ros2 run turtle_localization frontier
```

And finally, start the exploration node:

```bash
$ ros2 run turtle_localization explore
```

