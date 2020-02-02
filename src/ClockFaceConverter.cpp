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
#include <example/ClockFaceConverter.h>

// This class splits up a ClockFace message into vector3 messages so that you can visualize in Rviz
// Also shows how to use parameters and topic re-assignments
namespace arc
{
  ClockFaceConverter::ClockFaceConverter() :
  // I like use initializer lists for this
  // Keep in mind, they are initialized in their order in the header file, not as listed here
  // Main benefit is that your parameters can be constants
  m_nh{ros::NodeHandle()},
  m_pnh{ros::NodeHandle("~")},
  m_hourPub{},
  m_minutePub{},
  m_secondPub{},
  m_faceSub{},
  // get a private parameter, and use a default value if it isn't specified
  m_publishHours{m_pnh.param<bool>("publish_hours", "true")},
  m_publishMinutes{m_pnh.param<bool>("publish_minutes", "true")},
  m_publishSeconds{m_pnh.param<bool>("publish_seconds", "true")}
  {
    // set up our publishers
    if(m_publishHours)
    {
      m_hourPub = m_nh.advertise<visualization_msgs::Marker>("hour", 0);
    }

    if(m_publishMinutes)
    {
      m_minutePub = m_nh.advertise<visualization_msgs::Marker>("minute", 0);
    }

    if(m_publishSeconds)
    {
      m_secondPub = m_nh.advertise<visualization_msgs::Marker>("second", 0);
    }

    // set up our subscriber
    // drop old messages
    // send it to this instance's callback
    m_faceSub = m_nh.subscribe("face", 0, &ClockFaceConverter::clockFaceCallback, this);
  }

  void ClockFaceConverter::clockFaceCallback(const example_msgs::ClockFace& face)
  {
    if(m_publishHours)
    {
      visualization_msgs::Marker marker = generateMarker(face.hour);
      marker.color.r = 1.0;
      m_hourPub.publish(marker);
    }

    if(m_publishMinutes)
    {
      visualization_msgs::Marker marker = generateMarker(face.minute);
      marker.color.g = 1.0;
      m_minutePub.publish(marker);
    }

    if(m_publishSeconds)
    {
      visualization_msgs::Marker marker = generateMarker(face.second);
      marker.color.b = 1.0;
      m_secondPub.publish(marker);
    }
  }

  visualization_msgs::Marker ClockFaceConverter::generateMarker(const geometry_msgs::Vector3& vector)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time().now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.1; // shaft diameter
    marker.scale.y = 0.2; // head diameter
    marker.scale.z = 0.2; // head length

    geometry_msgs::Point head;
    head.x = 0;
    head.y = 0;
    head.z = 0;
    geometry_msgs::Point tail;
    tail.x = vector.x;
    tail.y = vector.y;
    tail.z = vector.z;
    // Arrays in messages are implemented as std::vectors
    marker.points.push_back(head);
    marker.points.push_back(tail);

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
  }
}
