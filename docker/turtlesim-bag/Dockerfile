FROM das:ros2-turtlesim
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins \
    nano vim \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /tmp/bag
WORKDIR /tmp/bag
CMD ["ros2", "bag", "record", "/turtle1/cmd_vel"]
# Command to run node to control turtlesim `ros2 run turtlesim turtle_teleop_key`