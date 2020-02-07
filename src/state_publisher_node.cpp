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

  // tracking variables
  const ros::Duration update_interval(ros::param::param<float>("~update_interval_sec", 1.0));
  char current_state = example_msgs::State::GOOD;
  ros::Time last_cycle_time = ros::Time::now();

  // main timer loop
  ros::Timer timer = nh.createTimer(ros::Duration( 1.0f / ros::param::param<float>("~update_rate_hz", 10.0f)),
    [&](const ros::TimerEvent& e)
    {
      // handle cycling
      // note, you can subtract two ros::Time objects and get a ros::Duration
      // you could also compare (ros::Time::now() - last_cycle_time).toSec() to a double
      if((e.current_real - last_cycle_time) >= update_interval)
      {
        last_cycle_time = e.current_real;
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
      state.header.stamp = e.current_real;

      // fill out the state
      state.state = current_state;

      // publish the message
      pub.publish(state);
    });

  ros::spin();

  return 0;
}
