#FROM moveit/moveit2:foxy-release 
FROM ros:foxy-ros-base	
LABEL maintainer="Emanuel Nunez S gmail dot com"
ENV HOME /root
WORKDIR $HOME
SHELL ["/bin/bash", "-c"]
ARG ROS_DISTRO=foxy

# Install dependencies
RUN apt-get update && apt-get install -y \
    	build-essential \
    	checkinstall \
    	zlib1g-dev \
    	libssl-dev \
    	wget

### Upgrade cmake to 3.22
# https://www.linuxcapable.com/install-cmake-on-ubuntu-20-04-lts/
RUN wget https://github.com/Kitware/CMake/releases/download/v3.22.2/cmake-3.22.2.tar.gz &&\
	tar -zxvf cmake-3.22.2.tar.gz	&&\
	cd cmake-3.22.2 	&&\
	sudo ./bootstrap	&&\
	sudo make		&&\
	sudo make install	

RUN apt-get update && apt-get install -y \ 
	python3-vcstool \
    	apt-utils \
    	dialog
    	
# install ros2 packages
RUN apt-get update && apt-get install -y \ 
	ros-foxy-control-msgs \
	ros-foxy-xacro \
	ros-foxy-angles \
	ros-foxy-ros2-control \
	ros-foxy-realtime-tools \
	ros-foxy-control-toolbox \
	ros-foxy-ros2-controllers \
	ros-foxy-joint-state-publisher \
	ros-foxy-joint-state-publisher-gui \
	ros-foxy-ament-cmake-clang-format \
	python3-colcon-common-extensions

# mystery dependencies
RUN apt-get update && apt-get install -y \ 
	ros-foxy-action-msgs \
	ros-foxy-geometry-msgs \
	ros-foxy-std-msgs \
	ros-foxy-rosidl-runtime-c \
	ros-foxy-builtin-interfaces \
	ros-foxy-unique-identifier-msgs \
	ros-foxy-trajectory-msgs

# Magic?
#RUN apt-get update &&  apt-get dist-upgrade -y
#RUN apt-get update &&  apt-get upgrade

### Install moveit2
RUN mkdir -p $HOME/ws_moveit2/src && \
	cd $HOME/ws_moveit2/src && \
	git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO &&\ 
#	for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done &&\
	rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
WORKDIR $HOME/ws_moveit2
RUN source /opt/ros/foxy/setup.bash && \
	colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

### Install moveit2_tutorial 
RUN mkdir -p $HOME/ws_moveit2_tut/src && \
	cd $HOME/ws_moveit2_tut/src && \
	git clone https://github.com/ros-planning/moveit2_tutorials -b foxy --depth 1 && \
	vcs import < moveit2_tutorials/moveit2_tutorials.repos && \
	apt-get update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y 
WORKDIR $HOME/ws_moveit2_tut
RUN source /opt/ros/foxy/setup.bash && \
	colcon build --mixin release

## Inject my modified scripts
# Controller file & cmake file
WORKDIR $HOME/ws_moveit2
COPY joystick_servo_example.cpp $HOME/ws_moveit2/src/moveit2/moveit_ros/moveit_servo/src/teleop_demo
COPY CMakeLists.txt 			$HOME/ws_moveit2/src/moveit2/moveit_ros/moveit_servo
RUN source /opt/ros/foxy/setup.bash && \ 
	colcon build --packages-select moveit_servo 
	
# launch file 
WORKDIR $HOME/ws_moveit2_tut
COPY servo_teleop.launch.py $HOME/ws_moveit2_tut/src/moveit2_tutorials/doc/realtime_servo/launch/
COPY devel_servo_teleop.launch.py $HOME/ws_moveit2_tut/src/moveit2_tutorials/doc/realtime_servo/launch/
COPY custom_controller.launch.py $HOME/ws_moveit2_tut/src/moveit2_tutorials/doc/realtime_servo/launch/
RUN source /opt/ros/foxy/setup.bash && \ 
	colcon build --packages-select moveit2_tutorials --mixin release

# Final prep
WORKDIR $HOME
RUN echo 'source /opt/ros/foxy/setup.sh && source $HOME/ws_moveit2/install/setup.bash && source $HOME/ws_moveit2_tut/install/setup.bash' >> $HOME/.bashrc


# RUN script
#CMD ["ros2", "launch", "moveit2_tutorials", "servo_teleop.launch.py"]
#RUN echo 'source /opt/ros/foxy/setup.sh && source $HOME/ws_moveit2/install/setup.bash && source $HOME/ws_moveit2_tut/install/setup.bash' >> $HOME/.bashrc

#RUN echo 'ros2 launch moveit2_tutorials servo_teleop.launch.py' >> $HOME/.bashrc
RUN echo 'ros2 launch moveit2_tutorials devel_servo_teleop.launch.py' >> $HOME/.bashrc
#RUN echo 'ros2 launch moveit2_tutorials custom_controller.launch.py' >> $HOME/.bashrc
 

