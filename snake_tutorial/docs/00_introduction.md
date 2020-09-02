# Snake Tutorial Introduction
This set of documents will walk you through the process of creating a simple,
snake-game controller. Before working through this tutorial, ensure you have
gone through the process of 
[setting up your ARC development environment](../../docs/00_introduction.md).

By the end of this tutorial, you should have an understanding of ROS and
how to use the development tools surrounding it.

## ROS Concepts
We discussed in the setup steps what ROS stands for and how it can be used to
develop robotic applications. Now we will take an in-depth look at how ROS
accomplishes this.

### ROS Filesystem
Software in ROS is organized in `packages`. Packages contain nodes, 
configuration files and more. When working on a project, you may have multiple
packages all responsible for various tasks. For instance, within this tutorial
you will be given a package for running the snake game (snakesim) and another
for learning how to control the snake (snake_tutorial).

Projects are often managed within a `catkin workspace`, which you setup
as part of your ARC development environment.

The simplified file structure for a workspace is as follows:
```
catkin_ws/
│
└───src/
│   │
│   └───example_package/
│
└───devel/
    │
    └─setup.bash
```
`src/` is where all packages are stored.

`devel/` contains a bash file for setting up your environment. Each time you
open your workspace in a new terminal you will need to source this setup file
by running:

```
source devel/setup.bash
```

Here is a simplified file structure for packages:
```
example_package/
│
└───src/
│   │
│   └───example_package/
│
└───nodes/
│
└───launch/
│
└───msg/
│
└───srv/
│
└───scripts/
│
└───CMakeLists.txt
│
└───package.xml
```

`src/example_package/` is where a majority of the magic happens.
This is where source files will be stored.

`nodes/` contains node types, which we will cover in the next section.

`launch/` contains launch files.

`msg/` contains message types.

`src/` contains service types.

`scripts/` contains executable scripts.

`CMakeLists.txt` is a build file used by your catkin workspace.

`package.xml` is a manifest that contains general package information.

### ROS Data Types
Now that we have a rough idea of where files are stored, lets look at what
these files actually are.

The types of data within ROS can be broken up into 4 main types:
- Nodes: These are independent processes that perform computations. These
 often relate nicely to physical components within a system, like a camera 
 or an arm for instance.
- Messages: In order for different nodes to communicate, they need messages
 to define the structure of the data they are sending. If a map node wanted
 to tell a navigation node which way to move, it would have to use a message
 (like a 3-point coordinate) to pass that data.
- Topics: Messages help define the type of data we are sending, but topics
 are what deliver the messages. Without topics, a node would never know which node to send it's messages to. Nodes can either publish
 or subscribe to a topic, which means they can either receive all messages
 on that topic, send messages to that topic, or both. 
- Services: Topics are nice because it means nodes can send information
 without worrying about if another node receives it. However sometimes it
 may be necessary that a node receives a response from another before moving on.
 This is where services come in; services allow for request / reply interactions.

Here's a helpful visualization of node communication:

![Node communication](images/node-communication.png)

## Verification
As mentioned, you should have already setup your ARC development environment.
Beyond that, just ensure your file structure matches the following:
```
catkin_ws/ <-- You should be here
│
└───src/
    │
    └───arc_tutorials/
       │
       └───clock_tutorial/ (deprecated tutorials)
       │
       └───docker/ (virtualization)
       │
       └───docs/ (setup instructions)
       │
       └───snake_tutorial/ (ROS tutorial)
       │
       └───snakesim/ (snake_tutorial backend)
```

## Instructions
To complete this tutorial, simply follow each provided document in order. Ensure
you are reading thoroughly, as these tutorials are packed with tons of
information.

If you run into issues, start by doing independent research. If you haven't
found a solution after sufficiently making an attempt on your own, then reach 
out within the [tutorial-assistance Slack channel](https://purduearc.slack.com/archives/C019N8EJRF0).