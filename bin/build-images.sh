#!/bin/bash
docker build docker/base-image/. -t das:ros2-base
docker build docker/rqt/. -t das:ros2-rqt
docker buildx build --platform linux/amd64,linux/arm64 -t petdog/das:latest-base --push -f docker/base-image/Dockerfile .
docker buildx build --platform linux/amd64,linux/arm64 -t petdog/das:latest --push -f docker/built-application/Dockerfile .

docker manifest create \
petdog/das:latest \
--amend petdog/das:latest-amd64 \
--amend petdog/das:latest-arm64v8