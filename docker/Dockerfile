
FROM ros:noetic-ros-base

RUN rosdep update \
    && sudo apt update \
    && sudo apt dist-upgrade -y

# install bootstrap tools
# install catkin
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    ros-$ROS_DISTRO-catkin \
    python3-catkin-tools \
    python3-pip \
    clang-format-10 \
    git \
    && rm -rf /var/lib/apt/lists/*

## Python3 libraries ##
RUN python3 -m pip install pip --upgrade && \
    pip install rospkg catkin-pkg pre-commit

ENV DEBIAN_FRONTEND noninteractive

RUN apt update 
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
               && mkdir ~/ros_ws && cd ~/ros_ws \
               && wstool init src \
               && wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall \
               && wstool update -t src \
               && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
               && catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN apt install -y ros-noetic-tf-conversions

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
               && cd ~/ros_ws \
               && catkin build"

RUN git config --global --add safe.directory /root/ros_ws/src/moveit
RUN git config --global --add safe.directory /root/ros_ws/src/moveit_resources

RUN echo "\nsource /root/ros_ws/devel/setup.bash\n" >> /root/.bashrc

RUN apt install -y catkin-lint ros-noetic-rqt ros-noetic-rqt-common-plugins 

# Traceability.
ARG DOCKERFILE_COMMIT_SHORT_SHA
ENV DOCKER_IMAGE_VERSION_O2AC_UR=${DOCKERFILE_COMMIT_SHORT_SHA}