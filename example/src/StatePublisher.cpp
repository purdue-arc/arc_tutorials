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

#include <example/StatePublisher.h>
#include <example_msgs/State.h>

namespace arc
{
  StatePublisher::StatePublisher() :
  m_nh{},
  m_pnh{"~"},
  m_heartbeatSub{},
  m_statePub{},
  m_stateTimer{},
  m_timeout_interval{m_pnh.param<float>("timeout_interval_sec", 1.0)},
  m_current_state{example_msgs::State::GOOD},
  m_last_cycle_time{ros::Time::now()}
  {
    // create a publisher on topic 'state', sending State messages, and have a queue of 1000
    m_statePub = m_nh.advertise<example_msgs::State>("state", 1000);

    // unless you use a multi-threaded ros spinner, callbacks are thread safe
    m_heartbeatSub = m_nh.subscribe("state/heartbeat", 0, &StatePublisher::heartbeatCallback, this);

    // Timer to publish state
    m_stateTimer = m_nh.createTimer(ros::Duration( 1.0f / m_pnh.param<float>("update_rate_hz", 10.0f)), &StatePublisher::timerCallback, this);
  }

  void StatePublisher::heartbeatCallback(const std_msgs::Empty& e)
  {
    // reset current state
    m_current_state = example_msgs::State::GOOD;
    m_last_cycle_time = ros::Time::now();
  }

  void StatePublisher::timerCallback(const ros::TimerEvent& e)
  {
    // handle cycling
    // note, you can subtract two ros::Time objects and get a ros::Duration
    // you could also compare (ros::Time::now() - last_cycle_time).toSec() to a double
    if(m_current_state != example_msgs::State::ABYSMAL &&
      (e.current_real - m_last_cycle_time) >= m_timeout_interval)
    {
      m_last_cycle_time = e.current_real;
      m_current_state++;
    }

    // create an instance of our message to send
    example_msgs::State state;

    // populate the time stamp with current time
    state.header.stamp = e.current_real;

    // fill out the state
    state.state = m_current_state;

    // publish the message
    m_statePub.publish(state);
  }
}
