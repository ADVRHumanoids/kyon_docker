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

# update keys
RUN sudo sh -c 'echo "deb http://xbot.cloud/xbot2/ubuntu/$(lsb_release -sc) /" > /etc/apt/sources.list.d/xbot-latest.list'
RUN wget -q -O - http://xbot.cloud/xbot2/ubuntu/KEY.gpg | sudo apt-key add -
RUN apt-get update && sudo apt-get install -y xbot2_desktop_full


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

RUN mkdir -p ws/src
# here it gets installed
RUN cd ~/ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_VERSION}/setup.sh && cd ~/ws && catkin_make install

# clone all the required packages
WORKDIR /home/user/ws/src
RUN git clone https://github.com/ADVRHumanoids/iit-kyon-ros-pkg
RUN git clone https://github.com/ADVRHumanoids/kyon_controller
RUN git clone https://github.com/ADVRHumanoids/kyon_codesign
RUN git clone https://github.com/FrancescoRuscelli/phase_manager



# source workspace for ROS_PACKAGE_PATH
RUN echo "source ~/ws/install/setup.bash" > .bashrc

# break cache for re-installation of casadi (to always download the last version)
# ARG CACHE_DATE="date"

# install horizon
RUN cd ~/ws/src && git clone -b receding_horizon https://github.com/ADVRHumanoids/horizon
RUN cd ~/ws/src/horizon && pip3 install -e .

