/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <cob_interactive_teleop/interactive_markers_tools.h>

namespace interactive_markers
{

void makeCircle(visualization_msgs::InteractiveMarkerControl &control,
                float radius,
                float width,
                std_msgs::ColorRGBA color
                )
{
  visualization_msgs::Marker marker;

  marker.pose.orientation = control.orientation;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;
  marker.color = color;

  int steps = 36;
  std::vector<geometry_msgs::Point> circle1, circle2;

  geometry_msgs::Point v1, v2;

  for (int i = 0; i < steps; i++)
  {
    float a = float(i) / float(steps) * M_PI * 2.0;

    v1.y = 0.5 * cos(a);
    v1.z = 0.5 * sin(a);

    v2.y = (1 + width) * v1.y;
    v2.z = (1 + width) * v1.z;

    circle1.push_back(v1);
    circle2.push_back(v2);
  }

  for (int i = 0; i < steps; i++)
  {
    int i1 = i;
    int i2 = (i + 1) % steps;

    marker.points.clear();
    marker.points.push_back(circle1[i1]);
    marker.points.push_back(circle2[i1]);
    marker.points.push_back(circle1[i2]);
    marker.points.push_back(circle2[i1]);
    marker.points.push_back(circle2[i2]);
    marker.points.push_back(circle1[i2]);

    control.markers.push_back(marker);
  }
}

}
