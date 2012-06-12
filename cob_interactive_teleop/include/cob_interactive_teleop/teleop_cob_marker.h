/******************************************************************************
 * \file
 *
 * $Id: teleop_cob_marker.h 869 2012-06-02 22:24:03Z spanel $
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

#ifndef TELEOPCOBMARKER_H_
#define TELEOPCOBMARKER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include <cob_interactive_teleop/interactive_markers_tools.h>
#include <cob_interactive_teleop/parameters_list.h>

namespace cob_interactive_teleop
{

const std::string MARKER_DRIVER_NAME      = "marker_driver";
const std::string MARKER_NAVIGATOR_NAME   = "marker_navigator";
const std::string CONTROL_MOVE_NAME       = "control_move";
const std::string CONTROL_STRAFE_NAME     = "control_strafe";
const std::string CONTROL_ROTATE_NAME     = "control_rotate";
const std::string CONTROL_NAVIGATION_NAME = "controle_naavigation";

typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;


/**
 * @brief Motion parameters for the interactive COB teleop.
 */
struct TeleopCOBParams
{
  double angular_scale;
  double linear_scale;
  double navigation_threshold;
  double rotate_on_move;
  double rotate_in_place;

  /**
   * @brief Constructor initializes all parameters to default values
   */
  TeleopCOBParams()
    : angular_scale(DEFAULT_ANGULAR_SCALE)
    , linear_scale(DEFAULT_LINEAR_SCALE)
    , navigation_threshold(DEFAULT_NAVIGATION_THRESHOLD)
    , rotate_on_move(DEFAULT_ROTATE_ON_MOVE)
    , rotate_in_place(DEFAULT_ROTATE_IN_PLACE)
  {}
};


/**
 * @brief This class handles COB driving using Interactive Markers.
 * @author Tomas Lokaj
 *
 * @see http://www.ros.org/wiki/cob_interactive_teleop
 */
class TeleopCOBMarker
{
public:
  /**
   * @brief Constructor
   */
  TeleopCOBMarker();

  /**
   * @brief Destructor
   */
  virtual ~TeleopCOBMarker()
  {
    server_->erase(MARKER_DRIVER_NAME);
    server_->erase(MARKER_NAVIGATOR_NAME);
    server_->applyChanges();
  }

  /**
   * @brief Returns reference to the motion parameters
   */
  TeleopCOBParams& getParams() { return params_; }

private:
  /**
   * @brief Markers feedback
   */
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * @brief Creates Interactive Markers
   */
  void createMarkers();

private:
  // Interactive Marker Server
  InteractiveMarkerServerPtr server_;

  // Movement publisher
  ros::Publisher pub_;

  // Node handler
  ros::NodeHandle n_;

  // Initial position
  geometry_msgs::Pose initial_pose_;

  // Teleop parameters
  TeleopCOBParams params_;
};

}

#endif /* TELEOPCOBMARKER_H_ */

