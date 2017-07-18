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
const std::string Z_POS_PARAM          = "z_pos";
const std::string DISABLE_DRIVER_PARAM = "disable_driver";

/**
 * Default parameter values
 */
const double DEFAULT_MAX_VEL_X       = 0.1;
const double DEFAULT_MAX_VEL_Y       = 0.1;
const double DEFAULT_MAX_VEL_TH      = 0.1;
const double DEFAULT_SCALE_LINEAR    = 0.1;
const double DEFAULT_SCALE_ANGULAR   = 0.1;
const double DEFAULT_Z_POS           = 0.15;

}

#endif // COB_INTERACTIVE_TELEOP_PARAMETERS_LIST_H

