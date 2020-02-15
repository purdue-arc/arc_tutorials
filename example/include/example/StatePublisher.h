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

#pragma once
#include <ros/ros.h>
#include <std_msgs/Empty.h>

// I like to put custom code in a namespace, so it doesn't mess up other included stuff or built in ros classes / functionality
namespace arc
{
  // Keeps track of state, and slowly times out if a heartbeat message stops
  class StatePublisher
  {
  public:
    // Constructor
    StatePublisher();

    // Destructor
    ~StatePublisher() = default;

    // Callback
    void heartbeatCallback(const std_msgs::Empty& e);
    void timerCallback(const ros::TimerEvent& e);

  private:
    ros::NodeHandle m_nh, m_pnh;
    ros::Subscriber m_heartbeatSub;
    ros::Publisher m_statePub;
    ros::Timer m_stateTimer;

    const ros::Duration m_timeout_interval;
    char m_current_state;
    ros::Time m_last_cycle_time;
  };
}
