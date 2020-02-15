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
#include <example_msgs/ClockFace.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

// I like to put custom code in a namespace, so it doesn't mess up other included stuff or built in ros classes / functionality
namespace arc
{
  // This class splits up a ClockFace message into vector3 messages so that you can visualize in Rviz
  // Also shows how to use parameters and topic re-assignments
  class ClockFaceConverter
  {
  public:
    // Constructor
    ClockFaceConverter();

    // Destructor
    ~ClockFaceConverter() = default;

    // Subscriber callback
    void clockFaceCallback(const example_msgs::ClockFace& face);

  private:
    // node handle
    ros::NodeHandle m_nh;

    // private node handle (used for parameters)
    ros::NodeHandle m_pnh;

    // publishers
    ros::Publisher m_hourPub;
    ros::Publisher m_minutePub;
    ros::Publisher m_secondPub;

    // subscribers
    ros::Subscriber m_faceSub;

    // parameters
    const bool m_publishHours;
    const bool m_publishMinutes;
    const bool m_publishSeconds;

    visualization_msgs::Marker generateMarker(const geometry_msgs::Vector3& vector);
  };
}
