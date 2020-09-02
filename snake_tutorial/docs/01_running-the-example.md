# Running the Snake Tutorial Example
Now that we have an understanding of higher-level ROS concepts, lets
see if we can apply them using this tutorial.

We will start by running the `snakesim` package to see what we will
be controlling. Run the following commands to open our 'dockerized'
environment:

```
./src/arc_tutorials/docker/docker-build.sh
```
then
```
./src/arc_tutorials/docker/docker-run.sh
```

You should now be running a shell within our ARC development container. To
verify that your catkin workspace is correct, run:

```
ls catkin_ws
```
If everything is correct, you should simply see a `src/` folder. Let's now
move into our workspace by running:

```
cd catkin_ws
```

Now we need to build our catkin workspace, which will configure our workspace
to run packages. We do this with the command:

```
catkin build
```

Now if we print our directory contents again:

```
ls
```

We should see a few new folders, including a `devel/` folder which (as
mentioned earlier) is important for configuring our environment.

The command to configure our environment is:

```
source devel/setup.bash
```

With our workspace and environment fully configured, we can now run the 
packages we have stored.

Lets start by moving into our package directory:

```
cd src/arc_tutorials
```

If we print our directory again, we should see 5 folders:
- `clock_tutorial/` a deprecated tutorial for ROS in C++
- `docker/` a set of tools to run ARC's docker environment
- `docs/` the collection of documents to follow this tutorial
- `snake_tutorial/` a ROS python package to teach basic control of the 
snake game
- `snakesim/` a ROS python package to run the snake game

To start this tutorial, we will run the snake simulation and control the
snake manually to understand how the game works. Then we will slowly
build our own controller to deepen our understanding of ROS.

In order to run the snake simulation, we will use what is known as a launch
file. It's not important for you to know how a launch file fully works at
the moment, but just know that it starts up ROS nodes in a set way so you
don't have to do it manually.

To run our snake sim launch file:

```
roslaunch snakesim.launch
```

Upon running this, two windows should appear. One shows a display of the
snake game and another is a control board for the snake (this is what we
will be replacing). Play around with it and see how far you can get.

>If nothing appears and you get an error message, it's most likely because
 X-forwarding is incorrectly setup. Return to 
 [setup tutorial #3](../../docs/03_arc-dev-env.md) for more detail.

Each time you interact with the control board, you are sending messages
and those messages are being received to change the speed and location of
the snake.

Now that we understand the game better, we can begin looking into making
our snake autonomous in the next tutorial.