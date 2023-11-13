# Use a build argument for the ROS version
ARG ROS_VERSION=noetic

# Use the specified ROS version as the base image
FROM ros:${ROS_VERSION}-robot

# after every FROM statement, all the ARGs are no longer available, this solves it:
ARG ROS_VERSION

# Update package lists and install dependencies
RUN apt-get update && apt-get install -y sudo build-essential git curl python3-tk python3-pip libjpeg-dev wget

# Upgrade pip and install the latest version of NumPy 
# only required for ROS noetic
RUN pip3 install --upgrade pip \
    && pip3 install --upgrade numpy

# Download and install CMake version 3.15
# only required for melodic
# RUN wget --no-check-certificate https://cmake.org/files/v3.15/cmake-3.15.0-Linux-x86_64.sh && \
#     chmod +x cmake-3.15.0-Linux-x86_64.sh && \
#     ./cmake-3.15.0-Linux-x86_64.sh --skip-license --prefix=/usr/local && \
#     rm cmake-3.15.0-Linux-x86_64.sh

# Clean up package lists to reduce image size
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Check NumPy version
RUN python3 -c "import numpy; print('NumPy version:', numpy.__version__)"

# Verify CMake installation
RUN cmake --version

# Install ROS catkin tools
RUN apt-get install -y ros-${ROS_VERSION}-catkin

# required for displaying matplotlib plots
ENV DISPLAY :1
# add user with sudo privileges which is not prompted for password
RUN adduser --disabled-password --gecos '' user
RUN adduser user sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER user
WORKDIR /home/user
ENV PATH="/home/user/.local/bin:${PATH}"

# Install ROS catkin tools
RUN sudo pip install hhcm-forest

# update keys and install xbot2
RUN sudo sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
RUN wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get install -y xbot2_desktop_full

# source xbot2 for ROS_PACKAGE_PATH
RUN echo ". /opt/xbot/setup.sh" >> ~/.bashrc

# install catkin workspace
RUN mkdir -p kyon_ws/src
RUN cd ~/kyon_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_VERSION}/setup.sh && cd ~/kyon_ws && catkin_make install

# clone all the required packages
WORKDIR /home/user/kyon_ws/src
RUN git clone https://github.com/ADVRHumanoids/iit-kyon-ros-pkg
RUN git clone https://github.com/ADVRHumanoids/kyon_controller
RUN git clone https://github.com/ADVRHumanoids/kyon_codesign


# source workspace for ROS_PACKAGE_PATH
RUN echo "source ~/kyon_ws/install/setup.bash" > .bashrc

# break cache for re-installation of casadi (to always download the last version)
# ARG CACHE_DATE="date"
# return to workdir

WORKDIR /home/user/
RUN mkdir ws
# create a new folder and install forest

RUN cd ~ws && forest init 
RUN cd ~ws && forest add-recipes git@github.com:ADVRHumanoids/multidof_recipes.git --clone-protcol=https


# install horizon
# RUN mkdir horizon_ws

# WORKDIR /home/user/horizon_ws
# RUN mkdir src build install

# RUN cd ~/horizon_ws/src && git clone -b receding_horizon https://github.com/ADVRHumanoids/horizon
# RUN cd ~/horizon_ws/src/horizon && pip3 install -e .

# RUN cd ~/horizon_ws/src && git clone https://github.com/FrancescoRuscelli/phase_manager
# RUN mkdir cd ../build/phase_manager && cmake ../../src/phase_manager -DCMAKE_INSTALL_PREFIX=/home/user/horizon_ws/install

