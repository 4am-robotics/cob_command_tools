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


#include <cob_interactive_teleop/teleop_cob_marker.h>

using namespace cob_interactive_teleop;

/**
 * @brief Just a main function which runs COB marker teleop node
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_interactive_teleop");

  new TeleopCOBMarker();

  ROS_INFO("COB Interactive Teleop is running...");

  ros::spin();
}

