FROM dastech/ros2-humble-base:latest

COPY . .
RUN rosdep update
RUN /ros_entrypoint.sh
RUN . /opt/ros/$ROS_DISTRO/setup.sh &&  \
    export _colcon_cd_root=/opt/ros/humble/ && \
    . /usr/share/colcon_cd/function/colcon_cd.sh && \
    colcon build --packages-select intra_process_demo