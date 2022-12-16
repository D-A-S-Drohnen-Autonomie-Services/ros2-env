cpunum = $(shell cat /proc/cpuinfo| grep "processor"| wc -l)

all: build install

.PHONY:build
build:
	colcon build --symlink-install

.PHONY:test
test:
	colcon test

.PHONY:install
install:
	. /install/setup.bash

.PHONY:clean
clean:
	rm -rf build
	rm -rf install
	rm -rf tmp/bag
	rm -rf log
