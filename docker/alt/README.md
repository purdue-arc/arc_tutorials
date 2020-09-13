# What is This?
This is an alternative catkin workspace for [ARC](https://www.purduearc.com/) members that includes zsh and a pre-loaded catkin workspace.

# Installation and Use
First, download the container by running `docker pull dgerblick/arc-dev:latest`.

Once it has finished downloading, run the following command to start the container:
```
docker run -dit \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    -v $XAUTH:/home/$USER/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --hostname arc-dev \
    --name arc-dev \
    dgerblick/arc-dev
```

You may then attach to the container by running `docker attach arc-dev`.

# Other Helpful Tips
The default password for the user is `boilerup`. You can change this by running `passwd` in the container

if you see `You cannot attach to a stopped container, start it first` when running `docker attach arc-dev`,
this means the container has stopped.  You can start and attach to it again by running `docker start -ai arc-dev`

You can see the status of all your docker containers by running `docker ps -a`.

You can add another shell prompt by running `docker exec -ti arc-dev zsh`.

