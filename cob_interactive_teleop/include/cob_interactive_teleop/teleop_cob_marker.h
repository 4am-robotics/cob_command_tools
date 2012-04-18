/******************************************************************************
 * \file
 *
 * $Id: teleop_cob_marker.h 649 2012-04-18 06:48:09Z spanel $
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
#include <but_cob_teleop/interactive_markers_tools.h>

using namespace interactive_markers;
using namespace visualization_msgs;
using namespace std;

namespace cob_interactive_teleop
{

#define ANGULAR_SCALE 2.2
#define LINEAR_SCALE 1.0

#define NAVIGATION_TRESHOLD 0.2
#define ROTATE_ON_MOVE 0.01
#define ROTATE 2.5

#define MARKER_DRIVER_NAME "marker_driver"
#define MARKER_NAVIGATOR_NAME "marker_navigator"
#define CONTROL_MOVE_NAME "control_move"
#define CONTROL_STRAFE_NAME "control_strafe"
#define CONTROL_ROTATE_NAME "control_rotate"
#define CONTROL_NAVIGATION_NAME "controle_naavigation"

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

/**
 * @brief This class handles COB driving using Interactive Markers.
 * @author Tomas Lokaj
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

private:
  /**
   * @brief Markers feedback
   */
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  /**
   * @brief Creates Interactive Markers
   */
  void createMarkers();

  InteractiveMarkerServerPtr server_; // Interactive Marker Server
  ros::Publisher pub_; // Movement publisher
  ros::NodeHandle n_; // Node handler
  geometry_msgs::Pose initial_pose_; // Initial position

};

}

#endif /* TELEOPCOBMARKER_H_ */
