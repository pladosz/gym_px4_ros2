FROM godhj/autolanding:latest

SHELL ["/bin/bash", "-c"]

WORKDIR /home/user/landing

ADD . /home/user/landing
	

RUN ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Gradle (Required to build Fast-RTPS-Gen)
RUN wget -q "https://services.gradle.org/distributions/gradle-6.3-rc-4-bin.zip" -O /tmp/gradle-6.3-rc-4-bin.zip \
	&& mkdir /opt/gradle \
	&& cd /tmp \
	&& unzip -d /opt/gradle gradle-6.3-rc-4-bin.zip \
	&& rm -rf /tmp/*

ENV PATH "/opt/gradle/gradle-6.3-rc-4/bin:$PATH"

# install Fast-RTPS dependencies
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev


# Intall foonathan_memory from source as it is required to Fast-RTPS >= 1.9
RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git /tmp/foonathan_memory \
	&& cd /tmp/foonathan_memory \
	&& mkdir build && cd build \
	&& cmake .. \
	&& cmake --build . --target install -- -j $(nproc) \
	&& rm -rf /tmp/*

# Fast-DDS (Fast-RTPS 2.0.2)
RUN git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.2 /tmp/FastDDS-2.0.2 \
	&& cd /tmp/FastDDS-2.0.2 \
	&& mkdir build && cd build \
	&& cmake -DTHIRDPARTY=ON -DSECURITY=ON .. \
	&& cmake --build . --target install -- -j $(nproc) \
	&& rm -rf /tmp/*

# Fast-RTPS-Gen 1.0.4
RUN git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 /tmp/Fast-RTPS-Gen-1.0.4 \
	&& cd /tmp/Fast-RTPS-Gen-1.0.4 \
	&& gradle assemble \
	&& gradle install \
	&& rm -rf /tmp/*


#install missing ros dependencies
RUN source /root/ros2_foxy/ros2-linux/local_setup.bash \
  && apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget\
  ros-foxy-rmw-fastrtps-cpp\
  ros-foxy-eigen3-cmake-module
# install Cyclone DDS dependencies
RUN source /root/ros2_foxy/ros2-linux/local_setup.bash \
  && pip3 install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  empy \
  pyros-genmsg 
RUN apt install --no-install-recommends -y \
  libcunit1-dev
#install gazebo ros packages
RUN mkdir -p ~/gazebo_ws/src
RUN cd ~/gazebo_ws \
    && wget https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/ros2/gazebo_ros_pkgs.repos\
    && vcs import src < gazebo_ros_pkgs.repos\
	&& vcs custom --args checkout foxy || true
	#ignore error during checkout of appropraite gazebo branch
RUN cd ~/gazebo_ws \ 
     && source /root/ros2_foxy/ros2-linux/local_setup.bash \
	 && rosdep install --from-paths src --ignore-src -r -y \
	 && colcon build --symlink-install
#install ros2-px4 package
RUN mkdir -p ~/px4_ros_com_ros2/src
RUN git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros2/src/px4_ros_com
RUN git clone https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros2/src/px4_msgs
#RUN cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts \
#    && source /root/ros2_foxy/ros2-linux/local_setup.bash \
#    && source ~/gazebo_ws/install/setup.bash \
#    && source build_ros2_workspace.bash --ros_distro foxy --ros_path /root/ros2_foxy/ros2-linux/local_setup.bash --verbose
