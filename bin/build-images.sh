#!/bin/bash
docker build docker/base-image/. -t das:ros2-base
docker build docker/turtlesim/. -t das:ros2-turtlesim
docker build docker/rqt/. -t das:ros2-rqt
docker build docker/turtlesim-bag/. -t das:ros2-bag