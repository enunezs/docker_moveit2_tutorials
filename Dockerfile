FROM moveit/moveit2:foxy-release 
LABEL maintainer="Emanuel Nunez S gmail dot com"
ENV HOME /root
WORKDIR $HOME
SHELL ["/bin/bash", "-c"]
ARG ROS_DISTRO=foxy


### Upgrade cmake to 3.22
# https://www.linuxcapable.com/install-cmake-on-ubuntu-20-04-lts/

# Install dependencies
RUN apt-get update && apt-get install -y \
    	build-essential \
    	checkinstall \
    	zlib1g-dev \
    	libssl-dev \
    	wget

# Install cmake 3.22
RUN wget https://github.com/Kitware/CMake/releases/download/v3.22.2/cmake-3.22.2.tar.gz &&\
	tar -zxvf cmake-3.22.2.tar.gz	&&\
	cd cmake-3.22.2 	&&\
	sudo ./bootstrap	&&\
	sudo make		&&\
	sudo make install	
	
### Install tutorial
# 
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
	ros-foxy-moveit \
	ros-foxy-ros2-controllers \
	ros-foxy-joint-state-publisher \
	ros-foxy-joint-state-publisher-gui \
	ros-foxy-ament-cmake-clang-format \
	python3-colcon-common-extensions


#RUN source /opt/ros/foxy/setup.bash && \

RUN mkdir -p $HOME/ws_moveit2/src && \
	cd $HOME/ws_moveit2/src && \
	git clone https://github.com/ros-planning/moveit2_tutorials -b foxy --depth 1 && \
	vcs import < moveit2_tutorials/moveit2_tutorials.repos && \
	apt-get update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y 
WORKDIR $HOME/ws_moveit2
RUN source /opt/ros/foxy/setup.bash && \
	colcon build --mixin release
	
WORKDIR $HOME

#RUN echo 'source $HOME/ws_moveit2/install/setup.bash' >> $HOME/.bashrc
RUN echo 'source /opt/ros/foxy/setup.sh && source $HOME/ws_moveit2/install/setup.bash' >> $HOME/.bashrc

CMD ["ros2", "launch", "moveit2_tutorials", "servo_teleop.launch.py"]

#RUN echo "source /etc/setup.bash" >> /etc/bash.bashrc
