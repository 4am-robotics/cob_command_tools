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
const std::string MAX_VEL_X_PARAM      = "max_vel_x";
const std::string MAX_VEL_Y_PARAM      = "max_vel_y";
const std::string MAX_VEL_TH_PARAM     = "max_vel_th";
const std::string SCALE_LINEAR_PARAM   = "scale_linear";
const std::string SCALE_ANGULAR_PARAM  = "scale_angular";

/**
 * Default parameter values
 */
const double DEFAULT_MAX_VEL_X       = 0.1;
const double DEFAULT_MAX_VEL_Y       = 0.1;
const double DEFAULT_MAX_VEL_TH      = 0.1;
const double DEFAULT_SCALE_LINEAR    = 0.1;
const double DEFAULT_SCALE_ANGULAR   = 0.1;

}

#endif // COB_INTERACTIVE_TELEOP_PARAMETERS_LIST_H

