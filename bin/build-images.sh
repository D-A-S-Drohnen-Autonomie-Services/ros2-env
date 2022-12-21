#!/bin/bash
docker build docker/base-image/. -t das:ros2-base
docker build docker/rqt/. -t das:ros2-rqt