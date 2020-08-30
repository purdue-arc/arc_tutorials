# Heading Controller
Since our snake recieves linear and angular velocity inputs, making a heading
controller seems like a good place to start. This will allow us to control the
heading of the snake using feedback control.

Feedback control (also called closed-loop control) means that we are using
sensor readings (in this case the output pose of the snake) in order to create
our control signal that we send to the snake. As a block diagram, it may look
like this:
![heading controller](images/heading-controller.png)

Let's go ahead and implement this controller in Python now.

We need to start by creating a new file. In the last tutorial, we were left with
the proper directory structure to start writing code. We need to make a new file
in the `nodes` directory to house our code. We'll call this file
`snake_position_controller`. 

Note that it is general practice not to add a `.py` file extension to nodes.
This is because the file name becomes the name of the node when building our
package with catkin. Ex: `roslaunch snake_controller snake_position_controller`
is cleaner than `roslaunch snake_controller snake_position_controller.py`.

Let's start the file by writing a shebang and docstring.
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""
```
A shebang is a one line comment at the start of the file that tells the
computer how to run it. We will always use `#!/usr/bin/env python` for Python
files. You can read more about them [here](https://en.wikipedia.org/wiki/Shebang_(Unix))

A docstring is a comment in triple double quotes (`"""`) that serves as a
comment for that file, class, function, or method. They are part of the PEP 8
style guide for Python, which we'll be following for our Python code. You can
read more about docstrings [here](https://www.python.org/dev/peps/pep-0257/#what-is-a-docstring),
and more about PEP 8 [here](https://www.python.org/dev/peps/pep-0008/#introduction).

Now, let's move on and start laying the foundation for our file by writing our
main function and creating a class to hold our logic.
```python
class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        pass

if __name__ == "__main__":
    SnakeHeadingController()
```
What we've done in the first part of this program is create a class called
`SnakeHeadingController`. A class lets you group methods and variables together
inside an object. It is a key part of Object Oriented Programming (OOO). It is
useful for us, because we're going to have several variables holding data that
we'll need to access from different methods.

This class has a docstring like before, and it only has one method defined,
`__init__`. This function is called when you create a new instance of that class
and is used for initialization. Right now, we've put `pass` inside, which is a
Python keyword meaning "do nothing." It is a handy placeholder because leaving
the body of that function blank would result in an error.

In the second part of the program, we've told the program what to do if it is
executed. This piece gets called when we tell ROS to launch the node. In this
case, it is creating an instance of our `SnakeHeadingController` class and
doing nothing else.

If you'd like, you can run this piece of code from your terminal with the
command:
```bash
python snake_heading_controller
```
A keen observer will notice that nothing happened. This is because our code
doesn't do anything yet. We call the second part of our code, which creates an
instance of a `SnakeHeadingController`, which gets initialized through the
`__init__` method, which does nothing.

Let's do our ROS specific setup now.

Let's modify the body of the `__init__` function to be the following. This is
replacing the `pass` keyword.
```python
        rospy.init_node('snake_heading_controller')

        rospy.spin()
```

Very importantly, we also need to tell Python to import the `rospy` module, so
that we have access to these functions. Add the following just below the
docstring:
```python
import rospy
```
An import command lets you pull in functionality from other Python files. This
is super useful to be able to re-use code and write small, module files. If you
look at the source of the `snakesim` package, you will see it is full of imports.

Here is our current file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""
import rospy

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        rospy.spin()

if __name__ == "__main__":
    SnakeHeadingController()
```


Full file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""

# Python
import math
from angles import shortest_angular_distance

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        self.heading_command = None
        self.linear_velocity_command = None
        self.angular_velocity = rospy.get_param('~angular_velocity', 6.28)

        # Publishers
        self.twist_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)
        rospy.Subscriber('controller/velocity', Float64, self.velocity_cb)

        rospy.spin()

    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        self.heading_command = heading_msg.data

    def velocity_cb(self, velocity_msg):
        """Callback for velocity command."""
        self.linear_velocity_command = velocity_msg.data

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        if (self.heading_command is not None
                and self.linear_velocity_command is not None):
            quat = (pose_msg.poses[0].orientation.x,
                    pose_msg.poses[0].orientation.y,
                    pose_msg.poses[0].orientation.z,
                    pose_msg.poses[0].orientation.w)

            __, __, heading = euler_from_quaternion(quat)
            error = shortest_angular_distance(heading, self.heading_command)
            angular_velocity = math.copysign(self.angular_velocity, error)

            twist_msg = Twist()
            twist_msg.linear.x = self.linear_velocity_command
            twist_msg.angular.z = angular_velocity
            self.twist_pub.publish(twist_msg)

if __name__ == "__main__":
    SnakeHeadingController()
```