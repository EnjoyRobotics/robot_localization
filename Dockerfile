FROM ros:galactic
ARG OVERLAY_WS=/opt/ros/overlay_ws


# Update packages list, download packages
RUN apt update --fix-missing -y
RUN apt install -y libboost-thread-dev
RUN apt install -y python3-pip
RUN pip3 install scipy


# Install external source
WORKDIR /$OVERLAY_WS/src
RUN git clone https://github.com/AlexKaravaev/csm.git
RUN git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher.git

# Install dependencies
WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -y \
      --from-paths src\
      --ignore-src

# Build source
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install \
        --packages-select csm ros2_laser_scan_matcher


# Clone source
WORKDIR /$OVERLAY_WS/src
COPY . ./robot_localization


# Install dependencies
WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -y \
      --from-paths src\
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build source
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install \
        --packages-select robot_localization


# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run launch file
CMD ros2 launch robot_localization ekf.launch.py
