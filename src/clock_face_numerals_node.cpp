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
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clock_face_numerals");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("numerals", 1000);

  const double radius = 1.1;
  const bool romanNumerals = ros::param::param("~use_roman_numerals", true);

  ros::Timer timer = nh.createTimer(ros::Duration( 1.0f ),
    [&](const ros::TimerEvent& e)
    {
      for(int i = 1; i <= 12; i++)
      {
        // create an instance of our message to send
        visualization_msgs::Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time().now();
        marker.id = i;  // Give each a unique id so they don't overwrite
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;    // keep on same plane
        marker.scale.z = 0.2;   // Text height
        marker.color.a = 1.0;   // Transparency
        marker.color.r = 1.0;   // Make white
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        if(romanNumerals)
        {
          switch (i) {
              case 1: marker.text = "I"; break;
              case 2: marker.text = "II"; break;
              case 3: marker.text = "III"; break;
              case 4: marker.text = "IV"; break;
              case 5: marker.text = "V"; break;
              case 6: marker.text = "VI"; break;
              case 7: marker.text = "VII"; break;
              case 8: marker.text = "VIII"; break;
              case 9: marker.text = "IX"; break;
              case 10: marker.text = "X"; break;
              case 11: marker.text = "XI"; break;
              case 12: marker.text = "XII"; break;
            }
          }
          else
          {
            char text[2];
            sprintf(text, "%d", i);
            marker.text = text;
          }
          const double angle = 2 * M_PI * i / 12.0;
          marker.pose.position.y = radius * -1.0 * sin(angle);
          marker.pose.position.z = radius * cos(angle);

          // publish the message
          pub.publish(marker);
      }
    });

  // handle callbacks continuously until process ends
  ros::spin();

  return 0;
}
