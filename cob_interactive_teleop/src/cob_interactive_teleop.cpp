/******************************************************************************
 * \file
 *
 * $Id: cob_interactive_teleop.cpp 674 2012-04-19 13:59:19Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 09/02/2012
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cob_interactive_teleop/teleop_cob_marker.h>

using namespace cob_interactive_teleop;

/**
 * @brief Just a main function which runs COB marker teleop node
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_interactive_teleop");

  TeleopCOBMarker * cobTeleop = new TeleopCOBMarker();

  ROS_INFO("COB Interactive Teleop is running...");

  ros::spin();
}

