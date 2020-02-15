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

namespace arc
{
  // This class decodes a ClockFace message
  class ClockFaceDecoder
  {
  public:
    // Constructor
    ClockFaceDecoder();

    // Destructor
    ~ClockFaceDecoder() = default;

    // Obtain the time from position
    static int getTime(const geometry_msgs::Vector3& hand, const int base);

    // Callback
    void clockFaceCallback(const example_msgs::ClockFace& face);

  private:
    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
  };
}
