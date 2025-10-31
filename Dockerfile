FROM ros:humble-ros-base-jammy AS aptgetter

# Install ROS 2 build tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-rclcpp-action \
    ros-humble-geometry-msgs \
    ros-humble-lifecycle-msgs \
    ros-humble-std-srvs \
    ros-humble-rosidl-default-generators \
    coinor-libcbc3 \
    coinor-libcbc-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies with compatible versions
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir "numpy==1.26.4" && \
    python3 -m pip install --no-cache-dir unified_planning

COPY action_planner /home/action_planner
WORKDIR /home/action_planner

# Make scripts executable
RUN chmod +x scripts/*.py

# Source ROS 2 environment and build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd src/harpia_msgs && \
    colcon build --symlink-install && \
    cd .."

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source src/harpia_msgs/install/setup.bash && \
    colcon build --symlink-install"

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/action_planner/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/action_planner/src/harpia_msgs/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]