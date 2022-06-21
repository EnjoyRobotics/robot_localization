FROM ros:galactic
ARG OVERLAY_WS=/opt/ros/overlay_ws


# Update packages list, download packages
RUN apt-get update --fix-missing && apt install -y ros-galactic-navigation2  \ 
  ros-galactic-nav2-bringup

# Clone source
WORKDIR /$OVERLAY_WS/src
RUN git clone https://github.com/AlexKaravaev/csm.git
COPY . ./robot_localization

RUN mv robot_localization/ros2_laser_scan_matcher ./ros2_laser_scan_matcher

# Install dependencies
WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -y \
      --from-paths src\
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build source
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run launch file
CMD ros2 launch robot_localization ekf.launch.py