FROM osrf/ros:melodic-desktop-full
RUN apt update && apt upgrade -y

RUN apt install -y sudo nano vim git \
    x11-apps iputils-ping \
    python-catkin-tools python-pygame

ARG USER=arc
ARG PW=password
ARG UID=1000
ARG GID=1000

RUN groupadd -o -g $GID $USER && \
    useradd -om -u $UID -g $GID -G sudo -s /bin/bash $USER && \
    echo "$USER:$PW" | chpasswd

RUN echo '\nsource /ros_entrypoint.sh\nset +e' >> home/$USER/.bashrc

USER $USER
WORKDIR /home/$USER
