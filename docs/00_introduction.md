# Introduction
This set of documents will guide you through setting up your computer with the
ARC development environment. This environment is created and run through Docker,
which is a lightweight virtual machine. It has very little overhead and greatly
simplifies the set up process for new users.

At the end of this guide, you will have the environment set up on your machine
and ready for use.

## Development Environment Explanation
The ARC development environment is an Ubuntu 18.04 instance with ROS Melodic
installed. It has a few other tools installed as well. You will access it
primarily through a shell (command line) interface, but you can also use an IDE
like Visual Studio Code to develop as if everything was installed on your
machine directly.

### ROS
ROS stands for Robot Operating System. It is an industry standard tool for
creating autonomous robots. Essentially, it is a framework to create a mesh of
nodes that work together to control your robot. It has an API for both C++
and Python. Through it, you have a common language to define messages that
are sent from one node to another (for example a path planning node can send
waypoint messages to a low level control node). The beauty of it is that all the
nodes can be modular and developed independently if you have a well defined
interface of messages.

Another really excellent reason to use ROS is the 2000+ number of existing
packages that can act as 'plug and play' into your existing network. If you have
a new sensor you're looking to try, someone has probably already written a ROS
driver for it. Tasks such as mapping, perception, and state estimation that many
robots need to do probably have multiple packages that you can experiment with
and pick the best one for your specific application. There are also pre-defined
messages for standard things like sensor data and control commands. For these
reasons it can be really useful for jump-starting a new project.

Through this club, you will be learning much more about ROS. The `snake_tutorial`
package will walk you through creating a few simple nodes that can be chained
together to control a snake to reach a goal. You will also have the opportunity
to expand on the very basic controller given to you in an open ended challenge
to create the highest performing AI for the snake.

A downside of ROS is that it works really well on Ubuntu, but isn't well
supported on other operating systems. This is why we must set up a very specific
development environment for the club.

For more information on ROS, check out their wiki: <http://wiki.ros.org/ROS/Introduction>

## Operating System
See the below instructions depending on what operating system you are running.

### Windows
This guide was mainly written for students using Windows 10. Historically, this
has been the dominant OS for our members, and it was also difficult to get
started on. Read and follow all set up documents in order. You will first 
install and configure WSL2, then set up Docker, then build and test the ARC
development environment.

Versions of Windows other than Windows 10 do not appear to be supported by
Docker, so therefore the ARC development environment is not supported on them.
You will need to upgrade to WIndows 10, dual boot, or run a Linux VM. That is
outside the scope of this document.

### Mac
Presently, this guide has not been tested on a Mac system. In theory it should
work fine because there is a version of Docker for Mac. You can skip directly
over the WSL2 guide (since it is Windows specific) and continue with Docker
setup. You will then build and test the ARC development environment.

### Linux
This guide has been tested to work on Ubuntu 18.04. If you have a different
distribution, it should also work without issue, provided there is a Docker
release for you. If you run Linux as a dual boot with one of the above operating
systems, it may be worth installing the ARC development environment in both. See
the notes on dual booting below. For setting up the environment, you can skip
directly over the WSL2 guide (since it is Windows specific) and continue with
Docker setup. You will then build and test the ARC development environment.

## Dual Boot Considerations
For the best possible experience, it may be worth looking into setting up a dual
boot. ROS was designed to run on Ubuntu, so running it natively without anything
extra in the way will give you the least hassle when actually trying to run
code. Setting up the dual boot can be quite a hassle for an inexperienced user,
which is why we recomend using WSL2 / Docker. This also saves you from needing
to restart your computer every time you want to switch modes.

Setting up a dual boot is outside the scope of this guide, but you should be
able to find plenty of resources online. You will want to install the version of
Ubuntu that is correct for the version of ROS you are intending to use. This
information can be found the [ROS wiki](http://wiki.ros.org/ROS/Installation)
Presently, the club is using ROS Melodic, which is designed to run on Ubuntu
18.04. You will then need to decide if you are running ROS natively or though
Docker. That is also outside the scope of this document.

You can also install a different version of Ubuntu for your dual boot, then use
Docker in order to run the version you need. This will likely give you better
performance than running Docker on Windows or Mac, and you can follow the Linux
set up steps noted above.
