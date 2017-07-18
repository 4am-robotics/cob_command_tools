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


#ifndef INTERACTIVE_MARKERS_TOOLS_H_
#define INTERACTIVE_MARKERS_TOOLS_H_

#include <std_msgs/ColorRGBA.h>
#include <interactive_markers/interactive_marker_server.h>

namespace interactive_markers
{
/**
 * @brief Makes circle from markers and assigns it to specified control
 * @param control is control to which the circle will be assigned
 * @param radius is circle's radius
 * @param width is circle's width
 * @param color is circle's color
 */
void makeCircle(visualization_msgs::InteractiveMarkerControl &control,
                float radius,
                float width,
                std_msgs::ColorRGBA color
                );

}

#endif /* INTERACTIVE_MARKERS_TOOLS_H_ */

