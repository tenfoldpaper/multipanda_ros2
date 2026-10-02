# =======================================================================================
# Docker Development Environment for multipanda_ros2
# =======================================================================================
FROM ros:humble

ENV DEBIAN_FRONTEND noninteractive
ARG USER_UID=1000
ARG USER_GID=1000
ARG HOME_DIR=/home/developer
ARG WS_NAME=multipanda_ws

# *********************************************************
# User Identity & Core Ubuntu Packages
# *********************************************************
RUN groupadd --gid $USER_GID developer || true && \
    useradd --uid $USER_UID --gid $USER_GID --create-home developer && \
    echo 'root:install' | chpasswd && \
    echo 'developer:123' | chpasswd && \
    apt-get update && apt-get install -y \
    software-properties-common build-essential cmake git wget curl zip sudo \
    nano net-tools lsof pulseaudio iputils-ping iproute2 libpoco-dev \
    && adduser developer sudo \
    # This line ensures the sudo group is active and requires the password you set
    && echo '%sudo ALL=(ALL:ALL) ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# Set GCC-11 as the default compiler
RUN apt-get update && apt-get install -y gcc-11 g++-11 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-11

# *********************************************************
# Python Environment & Libraries
# *********************************************************
RUN apt-get update && apt-get install -y python3-pip && \
    ln -s /usr/bin/python3 /usr/bin/python

# Remove pip warning update messages
ENV PIP_DISABLE_PIP_VERSION_CHECK 1

RUN python3 -m pip install -U \
    argcomplete pyserial \
    matplotlib scipy playsound panda-python pytest

# *********************************************************
# 3. Source Builds (MuJoCo, Eigen, Libfranka)
# *********************************************************

# Switch to root to install dependencies and system-level libraries
USER root
RUN mkdir -p $HOME_DIR/Libraries $HOME_DIR/source_code
WORKDIR $HOME_DIR/source_code

# MuJoCo 3.2.0
RUN sudo apt-get update && sudo apt-get install -y \
    libglfw3 libglfw3-dev libgl1-mesa-dev libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev ninja-build zlib1g-dev clang-12 && \
    git clone -b 3.2.0 https://github.com/google-deepmind/mujoco.git && \
    cd mujoco && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=$HOME_DIR/Libraries/mujoco && \
    cmake --build . --parallel $(nproc) && cmake --install .

# Eigen 3.3.9
RUN curl -L https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz -o eigen.tar.gz && \
    tar -xzf eigen.tar.gz && cd eigen-3.3.9 && mkdir build && cd build && \
    cmake .. && sudo make install

# Libfranka 0.9.2
RUN git clone https://github.com/frankaemika/libfranka.git && \
    cd libfranka && git checkout 0.9.2 && git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$HOME_DIR/Libraries/libfranka .. && \
    cmake --build . --parallel $(nproc) && cmake --install .

# Ensure the developer user owns the libraries we just built as root
RUN chown -R developer:developer $HOME_DIR/Libraries
# *********************************************************
# ROS2 Workspace & Special Repositories
# *********************************************************
# Create Workspace Structure
RUN mkdir -p $HOME_DIR/$WS_NAME/src
WORKDIR $HOME_DIR/$WS_NAME/src

# Install dqrobotics
RUN add-apt-repository ppa:dqrobotics-dev/release && apt-get update && apt-get install libdqrobotics

# Install the package dependencies
RUN apt-get update -y && apt-get install -y --allow-unauthenticated \
    software-properties-common \
    clang-14 \
    clang-format-14 \
    clang-tidy-14 \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    ros-humble-control-msgs \
    ros-humble-xacro \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-clang-format \
    ros-humble-ament-flake8 \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-angles \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \    
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-hardware-interface-testing \
    ros-humble-launch-testing \
    ros-humble-generate-parameter-library \
    ros-humble-controller-interface \
    ros-humble-ros2-control-test-assets \
    ros-humble-controller-manager \
    ros-humble-moveit \
    ros-humble-nav-msgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-camera-info-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-broadcaster \
    ros-humble-warehouse-ros-sqlite \
    ros-humble-joint-trajectory-controller \
    ros-humble-rqt \
    ros-humble-joint-trajectory-controller \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-controller-manager \
    ros-humble-rqt-robot-steering \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-graph \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-plot \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# *********************************************************
# Rosdep Initialization
# *********************************************************

# Initialize rosdep (Check if already initialized to avoid errors)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi
RUN rosdep update

# *********************************************************
# Environment Config
# *********************************************************
# Source ROS2 and set library paths in bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> $HOME_DIR/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/Libraries/libfranka/lib:$HOME/Libraries/mujoco/lib' >> $HOME_DIR/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=$HOME/Libraries/libfranka/lib/cmake:$HOME/Libraries/mujoco/lib/cmake' >> $HOME_DIR/.bashrc

# Ensure developer owns their home directory
RUN chown -R developer:developer $HOME_DIR

USER developer
WORKDIR $HOME_DIR/$WS_NAME/