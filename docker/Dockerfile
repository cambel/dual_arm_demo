
FROM ros:melodic-ros-base

RUN apt update && apt install -y ros-melodic-moveit

# install bootstrap tools
# install catkin
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    ros-$ROS_DISTRO-catkin \
    python-catkin-tools \
    python-pip \
    && rm -rf /var/lib/apt/lists/*

## Python libraries ##
RUN python -m pip install pip --upgrade && \
    pip install rospkg catkin-pkg

RUN echo "\nsource /root/ros_ws/devel/setup.bash\n" >> /root/.bashrc

# Traceability.
ARG DOCKERFILE_COMMIT_SHORT_SHA
ENV DOCKER_IMAGE_VERSION_O2AC_UR=${DOCKERFILE_COMMIT_SHORT_SHA}