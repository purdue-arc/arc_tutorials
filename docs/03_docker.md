# Docker
Docker is a lightweight virtual machine (VM) that will let you run the ARC
development environment with very little overhead. Docker is useful because of
how lightweight it is, and how these VM instances can be created
programatically. You will learn more about how to use Docker when setting up the
ARC development environment.

This document will guide you through setting up Docker on your machine.
See the below section depending on what OS you are using.

## Windows
Follow the [installation guide](https://docs.docker.com/docker-for-windows/install-windows-home/)
to set up Docker to use the WSL2 backend.

The ARC development environment is currently untested on Windows 10 Pro. There 
are alternate [installation instructions](https://docs.docker.com/docker-for-windows/install/)
provided, and it seems you will have the option to use the WSL2 backend. For the
sake of having a uniform configuration for the club, use the WSL2 backend. There
are [additional notes](https://docs.docker.com/docker-for-windows/wsl/) that may
be useful.

## Mac
The ARC development environment is completely untested on Mac, but it should
work. Follow the [installation guide](https://docs.docker.com/docker-for-mac/install/).

## Linux
The ARC development environment has been tested on Ubuntu 18.04. Other supported
distributions should work too. Follow the [installation guide](https://docs.docker.com/engine/install/ubuntu/)
for your distribution. In the case of Ubuntu, installing via the repository is
the preferred method of installation.
