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
    goal_pub = rospy.Publisher('controller/goal', Point, queue_size=1)

    # Subscribers
    rospy.Subscriber(
        'snake/goal',
        PointStamped,
        lambda msg, pub=goal_pub: pub.publish(msg.point)
    )

    rospy.spin()
```