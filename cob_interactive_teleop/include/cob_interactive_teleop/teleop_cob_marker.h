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
const std::string CONTROL_NAVIGATION_NAME = "controle_navigation";

typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;


/**
 * @brief Motion parameters for the interactive COB teleop.
 */
struct TeleopCOBParams
{
  double max_vel_x, max_vel_y, max_vel_th;
  double scale_linear, scale_angular;
  double z_pos;
  bool disable_driver;

  /**
   * @brief Constructor initializes all parameters to default values
   */
  TeleopCOBParams()
    : max_vel_x(DEFAULT_MAX_VEL_X)
    , max_vel_y(DEFAULT_MAX_VEL_Y)
    , max_vel_th(DEFAULT_MAX_VEL_TH)
    , scale_linear(DEFAULT_SCALE_LINEAR)
    , scale_angular(DEFAULT_SCALE_ANGULAR)
    , z_pos(DEFAULT_Z_POS)
    , disable_driver(false)
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

  /**
   * @brief Changes position of the marekrs to the default state
   */
  void reinitMarkers();

private:
  /**
   * @brief Markers feedback
   */
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * @brief Limits a given velocity
   */
  double limitVel(double vel, double limit);

  /**
   * @brief Gives the sign (1, -1 or 0) of value
   */
  int sign(double value);

  /**
   * @brief Creates Interactive Markers
   */
  void createMarkers();

private:
  // Interactive Marker Server
  InteractiveMarkerServerPtr server_;

  // Movement publisher
  ros::Publisher pub_;

  // Node handles
  ros::NodeHandle n_, pn_;

  // Initial position
  geometry_msgs::Pose initial_pose_;

  // Teleop parameters
  TeleopCOBParams params_;
};

}

#endif /* TELEOPCOBMARKER_H_ */

