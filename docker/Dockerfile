
FROM ros:noetic-ros-base

RUN apt update && apt install -y ros-$ROS_DISTRO-moveit

# install bootstrap tools
# install catkin
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    ros-$ROS_DISTRO-catkin \
    python3-catkin-tools \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

## Python3 libraries ##
RUN python3 -m pip install pip --upgrade && \
    pip install rospkg catkin-pkg

RUN echo "\nsource /root/ros_ws/devel/setup.bash\n" >> /root/.bashrc

# Traceability.
ARG DOCKERFILE_COMMIT_SHORT_SHA
ENV DOCKER_IMAGE_VERSION_O2AC_UR=${DOCKERFILE_COMMIT_SHORT_SHA}