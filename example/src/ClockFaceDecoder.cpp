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

#include <example/ClockFaceDecoder.h>
#include <std_msgs/String.h>

// This class decodes a ClockFace message
namespace arc
{
  ClockFaceDecoder::ClockFaceDecoder() :
  m_nh{},
  m_pub{},
  m_sub{}
  {
    m_pub = m_nh.advertise<std_msgs::String>("face/string", 0);
    m_sub = m_nh.subscribe("face", 0, &ClockFaceDecoder::clockFaceCallback, this);
  }

  // Obtain the time from position
  int ClockFaceDecoder::getTime(const geometry_msgs::Vector3& hand, const int base)
  {
    double angle = -atan2(hand.y, hand.z); //rotate so that 0 is at top
    if(angle < 0)
    {
      angle += 2*M_PI;
    }
    const double time = angle / (2 * M_PI) * base;
    return static_cast<int>(time + 0.5); //ensures rounding, not truncation
  }

  // Callback
  void ClockFaceDecoder::clockFaceCallback(const example_msgs::ClockFace& face)
  {
    const int hour = getTime(face.hour, 12);
    const int minute = getTime(face.minute, 60);
    const int second = getTime(face.second, 60);

    char time[8];

    sprintf(time, "%d:%.2d:%.2d", hour, minute, second);
    std_msgs::String msg;
    msg.data = time;
    m_pub.publish(msg);
  }
}
