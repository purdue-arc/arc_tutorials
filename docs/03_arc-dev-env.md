# ARC Development Environment
The ARC development environment is hosted in a Docker image. This means that it
is programatically built using a 'Dockerfile' and can be launched with very
little overhead as a Docker container.

This document will walk you through building the image, and running it as a
container.

## Building the Image
To build the image, you need to give step-by-step commands to Docker. This is
typically done through the use of a 'Dockerfile.' You will therefore need the
Dockerfile associated with the ARC development environment. If you have not
already clone this repository to your computer, do so now.

### Cloning the repository
If using Windows, you should clone the repository to the WSL2 filesystem. The
easiest way to do that is to simply use the WSL2 terminal while running the
following commands.

The convenience scripts provided assume that you will be using this environment
in the context of a catkin workspace. You will need to do that to run the
`snake_tutorial` later on. No need to worry about what that means exactly, but
for now, make the following directory structure before cloning the repo.
```bash
cd
mkdir -p arc_ws/src
cd arc_ws/src
git clone https://github.com/purdue-arc/arc_tutorials.git
```
Note that you may modify the first command if you would like the workspace in a
directory other than your home folder. The rest of the tutorial will assume it
is in your home folder when giving absolute `cd` commands, so modify those if
you choose a different directory.

If you recieve an error that git is not installed, you can install it with the
following command on Ubuntu (our WSL2 instance is Ubuntu):
```bash
sudo apt install -y git
```

Back to building the image.

Navigate to the `docker` directory and run the `docker-build.sh` script.
```bash
cd arc_tutorials/docker
./docker-build.sh
```
This script may take some time to run. Behind the scenes, it is telling Docker
to build a new image based off of our Dockerfile. Our Dockerfile is telling
Docker to pull an existing Ubuntu ROS image off of [DockerHub](https://hub.docker.com/r/osrf/ros/),
install updates, install some helpful tools, and set up a new user.

You will see a note about changing the password after it finishes. It should be
fine to leave the sudo password as is, but you can change it if you want. To
commit your changes, see the section below.

Let's check that the image is actually built properly:
```bash
docker image ls
```

That command tells Docker to list all the images on your machine. If you see one
named `arc-dev`, then it worked.

## Running as a Container
Now that the image is built, we can run it.

Again, there is a convenience script to make sure you pass all the appropriate
arguments to Docker.
```bash
./docker-run.sh
```

This script will run the `arc-dev` image, pass in arguments in order to forward
the X display server, make the catkin workspace a shared folder between the
host and the container, and name the container `arc-dev`. It will also run it
in foreground and temporary mode. This means your current terminal will now run
commands in the container, not on your host. When this original connection is
closed (exiting the window, CTRL+D, or `exit`), the container will stop and all
data will be lost. Data written to the shared folder will persist since it is
actually pointing to your host. Data written to anyehwere else (such as
installed programs) will be lost unless if you commit your changes.

To test that everything is working, let's try running ROS.
```bash
roscore
```

You will see some messages about starting a new ROS master, then you can kill
the process with `CTRL+C`. We didn't install ROS on your host machine, so the
fact that it ran there means the development environment is properly set up.

## Committing changes
Commiting changes allows you to save the current state of your running
container to the image (or a new image). This can be useful if are doing some
rapid iteration and testing and need to install a few extra files. Or if you
want to modify something outside of the shared directory like your `bashrc` or
password.

If these are more persistant changes that other users (like teammates
on your project) will want to have, a better solution is to create a project
specific Dockerfile. This is outside the scope of this document.

The command is `docker commit`  
You run it like so:
```bash
docker commit <container name> <image name>:<tag>
```

For example, to take the current container and overwrite the existing image:
```bash
docker commit arc-dev arc-dev
```
If no tag is specified, `:latest` is assumed.

You can use tags in order to save specific states or version of an image. That
is outside the scope of this document.

## Joining the Container in a New Terminal
Many times, you will want to have multiple terminals open in a container so that
you can run multiple programs at once. There is a convenience script you can use
in order to to this.

From the host, run the script:
```bash
./docker-join.sh
```

You can use `roscore` to verify everything is set up properly. Remember that
closing the _original_ terminal will end the container even if you have several
other terminals joined.

## Testing X Forwarding
X forwarding is how the display from the container is sent to your host. In the
Dockerfile, a small program called `xeyes` was installed. You can run it inside
the container to make sure your display works.
```bash
xeyes
```

You'll see a little pair of eyes follow your cursor around. You can X out of it
or hit `CTRL+C` in order to kill the program.