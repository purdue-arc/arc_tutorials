# Snake Tutorial Introduction
## Tutorial Overview
Before we dive into ROS, let's break down what we will be accomplishing.
Our end goal is to automate a game of Snake. If you have never heard of
Snake, then check out an example 
[here](https://www.google.com/search?client=firefox-b-1-d&q=snake+game). 
We have made a ROS adaptation of it, which is what you will be automating.

There are two parts to this tutorial:
- Controller walkthrough: Here we will walk you through the
  creation of your own snake-controller. This will teach the basics of
  ROS and principles related to automation.
- Controller expansion: By the end of the previous section, you will 
  have a simple but functional controller. To strengthen your ROS skills, 
  you will now look at improving the controller and seeing how high of 
  score your snake can get.

> Before continuing, ensure you have
gone through the process of
[setting up your ARC development environment](../../docs/00_introduction.md).

## Verification
Ensure your file structure matches the following:
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
We will explain the file structure in-depth later, so don't worry if it doesn't
make any sense now.

## Instructions
To complete this tutorial, simply follow each provided documents in order. Ensure
you are reading thoroughly, as these tutorials are packed with tons of
information.

If you run into issues, start by doing independent research. If you haven't
found a solution after sufficiently making an attempt on your own, then reach 
out within the [tutorial-assistance Slack channel](https://purduearc.slack.com/archives/C019N8EJRF0).
