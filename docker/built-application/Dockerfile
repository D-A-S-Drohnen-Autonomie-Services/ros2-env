FROM petdog/das:latest-base

COPY ./src ./src
COPY ./Makefile ./Makefile
RUN chown -R root:root .
RUN ls -lstr
RUN rosdep update
#SHELL ["/bin/bash", "-c"]
RUN /ros_entrypoint.sh
RUN make clean
RUN . /usr/share/colcon_cd/function/colcon_cd.sh
RUN export _colcon_cd_root=/opt/ros/humble/
#RUN ["bash", "-c" , "colcon build --packages-select intra_process_demo"]
RUN colcon build --packages-select intra_process_demo
