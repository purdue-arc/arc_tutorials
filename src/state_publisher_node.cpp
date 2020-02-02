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
#include <example_msgs/State.h>

// A simple, one file node that publishes a bogus state for our system
int main(int argc, char **argv)
{
  // name this node 'state_publisher'
  ros::init(argc, argv, "state_publisher");

  // create a node handle
  // this is needed to create publishers, subscribers, timers, etc
  ros::NodeHandle nh;

  // create a publisher on topic 'state', sending State messages, and have a queue of 1000
  ros::Publisher pub = nh.advertise<example_msgs::State>("state", 1000);

  // create a rate object to run at 10 Hz
  // you can also use a timer as demonstrated in clock_face_node
  ros::Rate loop(10.0);

  while(ros::ok())
  {
    // create an instance of our message to send
    example_msgs::State state;

    // populate the time stamp with current time
    state.header.stamp = ros::Time::now();

    // fill out the state using the defined constants
    state.state = example_msgs::State::ABYSMAL;

    // publish the message
    pub.publish(state);

    // handle callbacks, send messages, do the behind the scenes stuff
    ros::spinOnce();

    // block thread until rate is ready
    loop.sleep();
  }

  return 0;
}
