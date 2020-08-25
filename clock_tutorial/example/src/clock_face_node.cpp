// Copyright (c)  2020 Autonomous Robotics Club of Purdue

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <ros/ros.h>
#include <example_msgs/ClockFace.h>
#include <example/ClockFace.h>

// A slightly more complicated example that shows how to split functionality accross multiple files / classes
int main(int argc, char **argv)
{
  // name this node 'clock_face'
  ros::init(argc, argv, "clock_face");

  // create a node handle
  // this is needed to create publishers, subscribers, timers, etc
  ros::NodeHandle nh;

  // create a publisher on topic 'face', sending ClockFace messages, and drop old messages
  ros::Publisher pub = nh.advertise<example_msgs::ClockFace>("face", 0);

  // create a ClockFace object
  arc::ClockFace face;

  // create a timer to run at 100 Hz
  // timer can run on lambda function as written, or an external callback
  // rate can also be used as demonstrated in state_publisher_node
  ros::Timer timer = nh.createTimer(ros::Duration( 1.0f / 100.0f),
    [&](const ros::TimerEvent& e)
    {
      // create an instance of our message to send
      example_msgs::ClockFace face_msg;

      // populate the vectors using the time when our timer was called
      face.setTime(e.current_real);
      face_msg.hour = face.getHourHand();
      face_msg.minute = face.getMinuteHand();
      face_msg.second = face.getSecondHand();

      // publish the message
      pub.publish(face_msg);
    });

  // handle callbacks continuously until process ends
  ros::spin();

  return 0;
}
