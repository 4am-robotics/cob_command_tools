/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Michal Spanel (spanel@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 02/06/2012
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

#ifndef COB_INTERACTIVE_TELEOP_PARAMETERS_LIST_H
#define COB_INTERACTIVE_TELEOP_PARAMETERS_LIST_H

#include <string>

namespace cob_interactive_teleop
{

/**
 * Names of parameters
 */
const std::string ANGULAR_SCALE_PARAM = "cob_interactive_teleop/angular_scale";
const std::string LINEAR_SCALE_PARAM  = "cob_interactive_teleop/linear_scale";

/**
 * Default parameter values
 */
const double DEFAULT_ANGULAR_SCALE        = 0.5;
const double DEFAULT_LINEAR_SCALE         = 0.5;
const double DEFAULT_NAVIGATION_THRESHOLD = 0.2;
const double DEFAULT_ROTATE_ON_MOVE       = 0.01;
const double DEFAULT_ROTATE_IN_PLACE      = 0.1;

}

#endif // COB_INTERACTIVE_TELEOP_PARAMETERS_LIST_H

