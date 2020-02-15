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
#include <geometry_msgs/Vector3.h>
#include <boost/date_time/posix_time/posix_time.hpp>

// I like to put custom code in a namespace, so it doesn't mess up other included stuff or built in ros classes / functionality
namespace arc
{
  // This pretty silly class creates vectors to represent the hands of a clock
  class ClockFace
  {
  public:
    // Constructor
    ClockFace();

    // Destructor
    ~ClockFace() = default;

    // Set the current time
    void setTime(const ros::Time& time);

    // Obtain the hour hand position
    const geometry_msgs::Vector3 getHourHand();

    // Obtain the minute hand position
    const geometry_msgs::Vector3 getMinuteHand();

    // Obtain the second hand position
    const geometry_msgs::Vector3 getSecondHand();

  private:
    boost::posix_time::time_duration m_time;
  };
}
