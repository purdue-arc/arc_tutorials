FROM osrf/ros:melodic-desktop-full
RUN apt update && apt upgrade -y

RUN apt install -y sudo nano vim git \
    x11-apps iputils-ping \
    python-catkin-tools python-pygame \
    zsh wget 

ARG USER=arc
ARG PW=boilerup
ARG UID=1000
ARG GID=1000

RUN groupadd -o -g $GID $USER && \
    useradd -om -u $UID -g $GID -G sudo -s /bin/bash $USER && \
    echo "$USER:$PW" | chpasswd

RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.1/zsh-in-docker.sh)" -- \
       -t gentoo \
       -p alias-finder -p colored-man-pages -p git \
       -p https://github.com/zsh-users/zsh-autosuggestions \
       -p https://github.com/zsh-users/zsh-completions \
       -a "ZSH=/home/$USER/.oh-my-zsh" \
       -a "source /opt/ros/$ROS_DISTRO/setup.zsh" && \
    cp -r /root/. /home/$USER/ && \
    mkdir -p /home/$USER/catkin_ws/src && \
    chown -R $USER /home/$USER  

USER $USER
WORKDIR /home/$USER/catkin_ws
RUN zsh -c "source ~/.zshrc && catkin build"
    
WORKDIR /home/$USER

CMD ["zsh"]
