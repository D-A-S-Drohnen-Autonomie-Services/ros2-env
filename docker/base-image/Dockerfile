FROM ros:humble-perception
ENV ROS_VERSION=2 ROS_PYTHON_VERSION=3 ROS_DISTRO=humble
# Need to update later https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
ENV ROS_DOMAIN_ID=50

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-demo-nodes-cpp \
    ros-humble-composition \
    ros-humble-turtle-tf2-py \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-rviz2  \
    ros-humble-ros-testing  \
    ros-humble-rttest  \
    ros-humble-tlsf-cpp  \
    nano vim \
    && rm -rf /var/lib/apt/lists/* \
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc \
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

RUN mkdir /home/root
ENV HOME /home/root
WORKDIR /home/root/project
RUN rosdep update
