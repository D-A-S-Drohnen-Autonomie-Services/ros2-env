FROM dastech/ros2-humble-base:latest
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ~nros-humble-rqt* \
    nano vim \
    && rm -rf /var/lib/apt/lists/* \
CMD ["rqt"]