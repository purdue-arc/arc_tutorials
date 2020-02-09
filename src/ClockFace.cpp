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

#include <example/ClockFace.h>

// This pretty silly class creates vectors to represent the hands of a clock
namespace arc
{
  ClockFace::ClockFace()
  {
    m_time = ros::Time::now().toBoost().time_of_day();
  }

  // Set the current time
  void ClockFace::setTime(const ros::Time& time)
  {
    m_time = time.toBoost().time_of_day();
  }

  // Obtain the hour hand position
  const geometry_msgs::Vector3 ClockFace::getHourHand()
  {
    // EST is UTC minus 5
    const double angle = 2 * M_PI * static_cast<double>(m_time.hours() - 5) / 12.0;
    // handy way to debug. rosrun rqt_console rqt_console
    ROS_INFO_STREAM("angle hour is " << angle);
    geometry_msgs::Vector3 hour;
    hour.x = 0.0;
    hour.y = 0.75 * -1.0 * sin(angle);
    hour.z = 0.75 * cos(angle);
    return hour;
  }

  // Obtain the minute hand position
  const geometry_msgs::Vector3 ClockFace::getMinuteHand()
  {
    const double angle = 2 * M_PI * static_cast<double>(m_time.minutes()) / 60.0;
    geometry_msgs::Vector3 minute;
    minute.x = 0.0;
    minute.y = 1.0 * -1.0 * sin(angle);
    minute.z = 1.0 * cos(angle);
    return minute;
  }

  // Obtain the second hand position
  const geometry_msgs::Vector3 ClockFace::getSecondHand()
  {
    const double angle = 2 * M_PI * static_cast<double>(m_time.seconds()) / 60.0;
    geometry_msgs::Vector3 second;
    second.x = 0.0;
    second.y = 1.0 * -1.0 * sin(angle);
    second.z = 1.0 * cos(angle);
    return second;
  }
}
