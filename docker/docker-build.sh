#!/bin/bash

docker build --build-arg USER=$USER \
             --build-arg PW="robot" \
             --build-arg UID=$(id -u) \
             --build-arg GID=$(id -g) \
             -t arc-dev \
             .

echo "
ARC development image built as 'arc-dev'
sudo password in container is 'robot'.
Run 'sudo passwd' inside the container to change it,
then run 'docker commit <container-id>' in a new terminal to make persistent"
