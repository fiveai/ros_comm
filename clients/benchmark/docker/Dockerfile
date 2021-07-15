# syntax=docker/dockerfile:experimental

# WARNING: The top line must go first in the file and is required to
#          allow cloning the git repository through ssh. More details
#          can be found at: https://medium.com/@tonistiigi/build-secrets-and-ssh-forwarding-in-docker-18-09-ae8161d066
# To build this docker image run:
#    1. export DOCKER_BUILDKIT=1
#    2. docker build --ssh default --tag XXX .

# ***************************************************************************************************
# * Copyright Five AI 2021.
# * All rights reserved.
# ***************************************************************************************************

FROM osrf/ros:kinetic-desktop-full-xenial

RUN apt-get update && apt-get install -y \
            openssh-client \
            nano \
            htop \
            openssh-server \
            iputils-ping    \
            net-tools

RUN mkdir -p -m 0600 ~/.ssh && \ssh-keyscan github.com >> ~/.ssh/known_hosts
RUN mkdir /var/run/sshd
RUN echo 'root:abcde' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd
ENV NOTVISIBLE "in users profile"

RUN echo "export VISIBLE=now" >> /etc/profile
RUN echo "/usr/sbin/sshd" >> ~/.bashrc
RUN echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> ~/.bashrc
RUN echo "source /ros_comm/devel-release/setup.sh" >> ~/.bashrc
RUN echo "mkdir -p \${FIVESHM_HOME}" >> ~/.bashrc
# This line is need to workaround a roslaunch bug where multiple instances of roscore are launched 
RUN echo "export ROS_MASTER_URI=http://\$(uname -n):11311" >> ~/.bashrc
RUN echo "nano /ros_comm/src/ros_comm/clients/benchmark/launch/launch.xml" >> ~/.bash_history
RUN echo "catkin_make_isolated  \
--source ./src/  \
--build /ros_comm/build-release \
--devel /projects/ros_comm/devel-release \
--install-space /ros_comm/install-release  \
--pkg=benchmark --install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
roslaunch --screen -v  benchmark launch.xml pub_image_count:=2000 \
sub_stats_file_path:=/root/tmp use_case:=1p1s_same_host" >> ~/.bash_history

RUN echo "python2 /ros_comm/src/ros_comm/clients/benchmark/execute.py --use_case=1p1s_same_host --extra_params=''" >> ~/.bash_history


RUN --mount=type=ssh git clone git@github.com:fiveai/ros_comm.git /ros_comm/src/ros_comm && \
    cd /ros_comm/src/ros_comm     && \
    git pull                      && \
    git checkout fiveshm          && \
    cd /ros_comm

RUN  /bin/bash -c "source /opt/ros/kinetic/setup.sh   &&        \
    catkin_make_isolated                                        \
    --source        /ros_comm/src/                       \
    --build         /ros_comm/build-release              \
    --devel         /ros_comm/devel-release              \
    --install-space /ros_comm/install-release            \
    --install                                                   \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

WORKDIR /ros_comm

EXPOSE 22
