# This is a repository for a four-wheel-steering robot

Before doing any operation with the robot, execute the next command. This command should be executed only once after the robot is turned on.
This shell script creates a symbolic link to specific USB port.

```python
sudo bash create_sym_ln.sh
```

## 1. Manual mode

To operate the robot in manual mode, open a new terminal and execute the next two commands.

```python
source fws_robot/install/setup.bash
ros2 launch robot_launch fws_launch.py
```

Open another terminal and execute the next three commands.

```python
source fws_robot/install/setup.bash
cd ~/fws_robot/UI/
python3 controller.py
```

The controller will appear. Move the red dot to operate the robot. Note that the robot is able to run in "In-phase" mode only during manual operation. It will not run in "In-phase" mode during navigation.

## 2. Autonomous mode

### 2.1 Map creation

To start map creation, open a new terminal and launch cartographer.

```python
source fws_robot/install/setup.bash
ros2 launch navigation cartographer_slam.launch.py
```

Open another terminal and execute the next three commands just as we did in the "Manual mode" section.

```python
source fws_robot/install/setup.bash
cd ~/fws_robot/UI/
python3 controller.py
```

To save a created map.

```python
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 2.2 Navigation

To do robot navigation, open a new terminal and execute the next command.

```python
source fws_robot/install/setup.bash
ros2 launch navigation navigation_rf2o.py
```
