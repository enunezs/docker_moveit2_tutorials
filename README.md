# docker_moveit2_tutorials

A dockerized version of the [moveit2_tutorials](https://github.com/ros-planning/moveit2_tutorials). Components are manually isntalled to allow recompilation.



Simply run:

```
./dockerscript.sh
```

To start the environment.

# File injection

To prevent carrying copies of the entire environment, we recommend to load custom controllers via the Dockerscript.
Individual files can be replaced before then building:

```bash
## Inject to moveit2
WORKDIR $HOME/ws_moveit2
COPY joystick_servo_example.cpp $HOME/ws_moveit2/src/moveit2/moveit_ros/moveit_servo/src/teleop_demo
COPY CMakeLists.txt 			$HOME/ws_moveit2/src/moveit2/moveit_ros/moveit_servo
COPY panda_simulated_config.yaml $HOME/ws_moveit2/src/moveit2/moveit_ros/moveit_servo/config
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \ 
	colcon build --packages-select moveit_servo 
	
# Inject to moveit2 tutorials
WORKDIR $HOME/ws_moveit2_tut
COPY servo_teleop.launch.py $HOME/ws_moveit2_tut/src/moveit2_tutorials/doc/realtime_servo/launch/
COPY devel_servo_teleop.launch.py $HOME/ws_moveit2_tut/src/moveit2_tutorials/doc/realtime_servo/launch/
COPY custom_controller.launch.py $HOME/ws_moveit2_tut/src/moveit2_tutorials/doc/realtime_servo/launch/

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \ 
	colcon build --packages-select moveit2_tutorials --mixin release
```

For debugging purposes,  2 scripts have been included to start an instance of `rviz2_script.sh` and `rqt_script.sh`

# Todo: 
 - [ ] Reorganise file structure. Auto load files and put in folders
