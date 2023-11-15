# Use a build argument for the ROS version
ARG ROS_VERSION=noetic

# Use the specified ROS version as the base image
FROM ros:${ROS_VERSION}-robot

# after every FROM statement, all the ARGs are no longer available, this solves it:
ARG ROS_VERSION

# Update package lists and install dependencies
RUN apt-get update && apt-get install -y sudo build-essential gfortran git curl python3-tk python3-pip libjpeg-dev wget
RUN apt-get install -y libassimp-dev liblapack-dev libblas-dev libyaml-cpp-dev libmatio-dev
# require for cartesio_acceleration_support
RUN apt-get install -y ros-${ROS_VERSION}-interactive-markers ros-${ROS_VERSION}-moveit
# required for casadi python bindings
RUN apt-get install -y swig
# required for realsense
RUN apt-get install -y ros-noetic-gazebo-ros-pkgs

# Install ROS catkin tools
RUN apt-get install -y ros-${ROS_VERSION}-catkin

# Upgrade pip and install the latest version of NumPy 
# only required for ROS noetic
RUN pip3 install --upgrade pip \
    && pip3 install --upgrade numpy

RUN pip3 install hhcm-forest

# required packages
RUN pip3 install scipy numpy_ros matplotlib

# Download and install CMake version 3.15
# only required for melodic
# RUN wget --no-check-certificate https://cmake.org/files/v3.15/cmake-3.15.0-Linux-x86_64.sh && \
#     chmod +x cmake-3.15.0-Linux-x86_64.sh && \
#     ./cmake-3.15.0-Linux-x86_64.sh --skip-license --prefix=/usr/local && \
#     rm cmake-3.15.0-Linux-x86_64.sh

# Check NumPy version
RUN python3 -c "import numpy; print('NumPy version:', numpy.__version__)"

# Verify CMake installation
RUN cmake --version

# required for displaying matplotlib plots
ENV DISPLAY :1
# add user with sudo privileges which is not prompted for password
RUN adduser --disabled-password --gecos '' user
RUN adduser user sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER user
WORKDIR /home/user
ENV PATH="/home/user/.local/bin:${PATH}"

# set up forest for installation

RUN mkdir forest_ws

WORKDIR /home/user/forest_ws
RUN forest init && forest add-recipes git@github.com:ADVRHumanoids/multidof_recipes.git --clone-protocol https

# install horizon
RUN forest grow horizon --clone-protocol https -j7

# WORKDIR /home/user/
# update keys and install xbot2
RUN sudo sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
RUN wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get install -y xbot2_desktop_full

# install catkin workspace
WORKDIR /home/user/
RUN mkdir -p kyon_ws/src
RUN cd ~/kyon_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_VERSION}/setup.sh && cd ~/kyon_ws && catkin_make install

# this order of sourcing is required, as catkin is crazy:

# source workspace for ROS_PACKAGE_PATH
RUN echo "source ~/kyon_ws/devel/setup.bash" >> ~/.bashrc

# source xbot2 for ROS_PACKAGE_PATH
RUN echo ". /opt/xbot/setup.sh" >> ~/.bashrc

# source forest
RUN echo "source ~/forest_ws/setup.bash" >> ~/.bashrc

# restart bash to make the source effective
SHELL ["bash", "-ic"]

# install iit-kyon-pkg and dependent packages
RUN sudo apt-get update
WORKDIR /home/user/forest_ws
RUN forest grow iit-kyon-ros-pkg --clone-protocol https -j7
RUN forest grow realsense_gazebo_description --clone-protocol https -j7
RUN forest grow iit-dagana-ros-pkg --clone-protocol https -j7

# install cartesio_acceleration_support
RUN cd ~/forest_ws && forest grow cartesio_acceleration_support --clone-protocol https -j7

# break cache for re-installation of kyon (to always download the last version)
ARG CACHE_DATE="date"

# clone all the required packages
WORKDIR /home/user/kyon_ws/src
RUN git clone -b deliverable https://github.com/ADVRHumanoids/kyon_controller
RUN git clone -b deliverable https://github.com/ADVRHumanoids/kyon_codesign


# return to workdir
WORKDIR /home/user/kyon_ws
RUN catkin_make

# roslaunch kyon_codesign lower_body_action.launch action:=trot

# pip install numpy_ros scipy
# # RUN cd ~/horizon_ws/src && git clone https://github.com/FrancescoRuscelli/phase_manager
# # RUN mkdir cd ../build/phase_manager && cmake ../../src/phase_manager -DCMAKE_INSTALL_PREFIX=/home/user/horizon_ws/install

# Clean up package lists to reduce image size
# RUN apt-get clean && \
#     rm -rf /var/lib/apt/lists/*

