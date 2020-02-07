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

  // tracking variables
  const ros::Duration update_interval(ros::param::param<float>("~update_interval_sec", 1.0));
  char current_state = example_msgs::State::GOOD;
  ros::Time last_cycle_time = ros::Time::now();

  while(ros::ok())
  {
    // why I like timers, e.current_real is defined
    const ros::Time now = ros::Time::now();

    // handle cycling
    // note, you can subtract two ros::Time objects and get a ros::Duration
    // you could also compare (ros::Time::now() - last_cycle_time).toSec() to a double
    if((now - last_cycle_time) >= update_interval)
    {
      last_cycle_time = now;
      current_state++;
      // handle overflow
      if(current_state > example_msgs::State::ABYSMAL)
      {
        current_state = example_msgs::State::GOOD;
      }
    }

    // create an instance of our message to send
    example_msgs::State state;

    // populate the time stamp with current time
    state.header.stamp = now;

    // fill out the state using the defined constants
    state.state = current_state;

    // publish the message
    pub.publish(state);

    // handle callbacks, send messages, do the behind the scenes stuff
    ros::spinOnce();

    // block thread until rate is ready
    loop.sleep();
  }

  return 0;
}
