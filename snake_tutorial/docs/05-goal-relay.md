# Heading Controller
## Overview
Our snake is now controlled by position. If we want to keep things really
simple, the snake can just chase the goal with no regard for walls or its own
body. You'll find it is pretty effective at the start of the game, but the
approach has some very clear flaws once the snake builds up any decent length.

This simple relay is what we'll develop now. In the future, you can expand on
this controller by fixing these problems. We'll give you a few more notes on
that in the next document.

## Creating the Program
This is going to start like the last program in creating the file, shebang, and
docstring. However, things are going to be different once we start writing
code. Go ahead and do all the above, and write an empty `main` function.

You should have something like this:
```python
#!/usr/bin/env python
"""Node to relay goal position to the position controller.

License removed for brevity
"""

if __name__ == "__main__":
    pass
```

We _could_ create a class like we've done in the past. However, this node is
going to be _really_ simple. We don't have any persistent data to keep track of
like commanded headings or positions. We'll also only have a single subscriber,
where we can put all of the logic. The logic won't even be anything spectacular.
It will simply be publishing the data from a PointStamped message as a Point
type. For something really small like this, it can be appropriate to skip over
creating a class, and put everything in `main`.

Look at the below program to see for yourself:
```python
#!/usr/bin/env python
"""Node to relay goal position to the position controller.

License removed for brevity
"""

# ROS
import rospy
from geometry_msgs.msg import PointStamped, Point

if __name__ == "__main__":
    rospy.init_node('snake_goal_relay')

    # Publishers
    goal_pub = rospy.Publisher('controller/position', Point, queue_size=1)

    # Subscribers
    rospy.Subscriber(
        'snake/goal',
        PointStamped,
        lambda msg, pub=goal_pub: pub.publish(msg.point)
    )

    rospy.spin()
```

We've used something called a lambda in order to put the logic right into the
call to create a subscriber. Rather than include the name of a function, we used
the `lambda` keyword, which lets us put the arguments separated by commas, then
the logic after a semicolon (:). Note that you can also pass in extra arguments
and even assign them using an equal sign (=) in the argument list.

You can learn more about lambdas [online](https://www.w3schools.com/python/python_lambda.asp).

## Updating Launch Files and Testing
Updating the launch file is no different from the position controller. Give it a
shot and see if the snake runs. Remember to `catkin build` and source!

You should have gotten something like this:
```xml
<node type="snake_goal_relay" pkg="snake_tutorial" name="snake_goal_controller"/>
```

You will find you no longer need to manually plug in data in order to test.
Instead the snake controller is done, and will run automatically. Use `rqt_plot`
to see how all the nodes work together!

## Final Notes
Congratulations, you just finished writing a basic controller for the snake!
Hopefully you learned a great deal about how ROS works, and how to write nodes
in Python. You should be confident to experiment with your own ideas in to
improve upon (or completely rewrite!) the controller we've just finished making.

Below, you'll see the final file we developed. Feel free to experiment with
making changes to this node, any of the previous ones, or start making your own.
There is some information to help you in the next document.

### Full File for Reference:
```python
#!/usr/bin/env python
"""Node to relay goal position to the position controller.

License removed for brevity
"""

# ROS
import rospy
from geometry_msgs.msg import PointStamped, Point

if __name__ == "__main__":
    rospy.init_node('snake_goal_relay')

    # Publishers
    goal_pub = rospy.Publisher('controller/position', Point, queue_size=1)

    # Subscribers
    rospy.Subscriber(
        'snake/goal',
        PointStamped,
        lambda msg, pub=goal_pub: pub.publish(msg.point)
    )

    rospy.spin()
```
