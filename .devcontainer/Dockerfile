FROM ros:jazzy
ARG USERNAME=user
ARG USER_UID=9999
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

# Set up apt dependencies for ros2 packages
RUN apt-get install -y ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-controller-interface ros-$ROS_DISTRO-realtime-tools \
    ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-joint-trajectory-controller ros-$ROS_DISTRO-joint-state-broadcaster

# Set up dependencies for moveit (ros-$ROS_DISTRO-moveit currently not working)
RUN apt-get install -y ros-$ROS_DISTRO-moveit-core \
    ros-$ROS_DISTRO-moveit-planners \
    ros-$ROS_DISTRO-moveit-plugins \
    ros-$ROS_DISTRO-moveit-ros-planning \
    ros-$ROS_DISTRO-moveit-ros-warehouse \
    ros-$ROS_DISTRO-moveit-ros-robot-interaction \
    ros-$ROS_DISTRO-moveit-ros-planning-interface \
    ros-$ROS_DISTRO-moveit-ros-visualization \
    ros-$ROS_DISTRO-moveit-ros-move-group \
    ros-$ROS_DISTRO-moveit-setup-assistant \
    ros-$ROS_DISTRO-moveit-planners-chomp \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-pinocchio

######################
# OTHER DEPENDENCIES #
######################

# Set up other apt dependencies
RUN apt-get install -y libyaml-cpp-dev libwebsocketpp-dev libjsoncpp-dev

# [Optional] Set the default user.
USER $USERNAME

# Source ros2 to .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]
