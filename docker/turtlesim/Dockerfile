FROM das:ros2-base
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-turtlesim \
    nano vim \
    && rm -rf /var/lib/apt/lists/*

CMD ["ros2", "run", "turtlesim", "turtlesim_node"]
# Command to run node to control turtlesim `ros2 run turtlesim turtle_teleop_key`