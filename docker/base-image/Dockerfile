FROM ros:humble-perception
ENV ROS_VERSION=2 ROS_PYTHON_VERSION=3 ROS_DISTRO=humble
# Need to update later https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
ENV ROS_DOMAIN_ID=50

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    nano vim \
    && rm -rf /var/lib/apt/lists/* \

RUN groupadd -g 1000 ros
RUN useradd -d /home/ros -s /bin/bash -m ros -u 1000 -g 1000
USER ros
ENV HOME /home/ros