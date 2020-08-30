```python
#!/usr/bin/env python
"""Node to control the position of the snake.

License removed for brevity
"""

# Python
import math

# ROS
import rospy
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Float64

class SnakePositionController(object):
    """Simple position controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_position_controller')

        self.goal = None

        # Publishers
        self.heading_pub = rospy.Publisher('controller/heading', Float64, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.snake_cb)
        rospy.Subscriber('controller/goal', Point, self.goal_cb)

        rospy.spin()

    def goal_cb(self, point_msg):
        """Callback for goal."""
        self.goal = (point_msg.x, point_msg.y)

    def snake_cb(self, pose_msg):
        """Callback for poses from the snake."""
        if self.goal is not None:
            goal = self.goal
            pose = (pose_msg.poses[0].position.x, pose_msg.poses[0].position.y)

            heading = math.atan2(goal[1] - pose[1], goal[0] - pose[0])
            self.heading_pub.publish(heading)

if __name__ == "__main__":
    SnakePositionController()
```