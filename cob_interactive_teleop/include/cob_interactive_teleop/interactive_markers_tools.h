/******************************************************************************
 * \file
 *
 * $Id: interactive_markers_tools.h 624 2012-04-16 14:05:56Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11/02/2012
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
void makeCircle(visualization_msgs::InteractiveMarkerControl &control, float radius, float width,
                std_msgs::ColorRGBA color);

}

#endif /* INTERACTIVE_MARKERS_TOOLS_H_ */
