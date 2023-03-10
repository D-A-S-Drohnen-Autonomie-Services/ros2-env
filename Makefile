cpunum = $(shell cat /proc/cpuinfo| grep "processor"| wc -l)
SHELL := /bin/bash

all: build install

.PHONY:build
build:
	colcon build --symlink-install

.PHONY:test
test:
	colcon test

.PHONY:install
install:
	source install/setup.bash

.PHONY:clean
clean:
	rm -rf build
	rm -rf install
	rm -rf tmp/bag
	rm -rf log

.PHONY:deps
deps:
	rosdep update && rosdep install --from-paths src --ignore-src -r -y