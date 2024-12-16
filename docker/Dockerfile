FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install dependencies and tools
RUN apt-get update && apt-get install -y \
    git wget curl python3-pip software-properties-common ros-dev-tools iputils-ping net-tools ufw vim gedit iproute2\
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir setuptools==58.2.0 colcon-common-extensions 

# Add Gazebo repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-control* \
    ros-humble-ros2-control* \
    ros-humble-moveit* \
    ros-humble-ros-ign* \
    ros-humble-joint-state-publisher-gui \
    ros-humble-kinematics-interface-kdl \
    ros-humble-rqt-joint-trajectory-controller \
    ~nros-humble-rqt* \
    ignition-fortress \
    ros-humble-rmw-cyclonedds-cpp \ 
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y

# Create a non-root user
RUN useradd -m -s /bin/bash panos && \
echo "panos ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/panos

# Set up the user's environment
COPY ./.bashrc /home/panos/.bashrc
RUN chown panos:panos /home/panos/.bashrc



# Set the working directory
WORKDIR /home/panos

# Switch to the non-root user
USER panos

RUN rosdep update

RUN echo "hi"
COPY ./config/laptop/cyclonedds.xml /cyclonedds.xml
RUN sudo mv /cyclonedds.xml /home/panos/.ros/

# Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN sudo chmod 755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
