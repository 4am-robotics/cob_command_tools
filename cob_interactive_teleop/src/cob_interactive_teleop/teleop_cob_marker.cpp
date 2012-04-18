/******************************************************************************
 * \file
 *
 * $Id: teleop_cob_marker.cpp 649 2012-04-18 06:48:09Z spanel $
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

namespace cob_interactive_teleop
{

TeleopCOBMarker::TeleopCOBMarker()
{
  server_.reset(new InteractiveMarkerServer("but_cob_interactive_teleop", "", false));
  pub_ = n_.advertise<geometry_msgs::Twist> ("base_controller/command", 1);

  initial_pose_ = geometry_msgs::Pose();
  initial_pose_.position.z = 0.05;

  createMarkers();

  server_->applyChanges();
}

void TeleopCOBMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  geometry_msgs::Twist twist;

  // Move forward or backward
  if (feedback->control_name == CONTROL_MOVE_NAME)
  {
    twist.linear.x = LINEAR_SCALE * feedback->pose.position.x;
  }
  // Strafe left or right
  else if (feedback->control_name == CONTROL_STRAFE_NAME)
  {
    twist.linear.y = LINEAR_SCALE * feedback->pose.position.y;
  }
  // Rotate
  else if (feedback->control_name == CONTROL_ROTATE_NAME)
    twist.angular.z = ANGULAR_SCALE * tf::getYaw(feedback->pose.orientation);
  // Move to position
  else if (feedback->control_name == CONTROL_NAVIGATION_NAME)
  {
    // Move towards the position and rotate slightly if necessary
    if (fabs(feedback->pose.position.y) <= NAVIGATION_TRESHOLD)
    {
      twist.linear.x = LINEAR_SCALE * feedback->pose.position.x;
      if (feedback->pose.position.y > 0)
        twist.angular.z = ROTATE_ON_MOVE;
      else if (feedback->pose.position.y < 0)
        twist.angular.z = -ROTATE_ON_MOVE;
    }
    // Just rotate
    else
    {
      if (feedback->pose.position.y > 0)
        twist.angular.z = ROTATE;
      else if (feedback->pose.position.y < 0)
        twist.angular.z = -ROTATE;
    }
  }
  else
    return;

  pub_.publish(twist);

  server_->setPose(MARKER_DRIVER_NAME, initial_pose_);
  server_->setPose(MARKER_NAVIGATOR_NAME, initial_pose_);
  server_->applyChanges();
}

void TeleopCOBMarker::createMarkers()
{
  InteractiveMarker marker_driver;
  marker_driver.name = MARKER_DRIVER_NAME;
  marker_driver.header.frame_id = "/base_link";
  marker_driver.header.stamp = ros::Time::now();
  marker_driver.pose = initial_pose_;
  marker_driver.scale = 1.5;

  InteractiveMarkerControl control;
  control.name = CONTROL_ROTATE_NAME;
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.orientation.w = 1;
  marker_driver.controls.push_back(control);

  control.name = CONTROL_MOVE_NAME;
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.orientation.w = 1;
  marker_driver.controls.push_back(control);

  control.name = CONTROL_STRAFE_NAME;
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.orientation.w = 1;
  marker_driver.controls.push_back(control);

  InteractiveMarker marker_navigator;
  marker_navigator.name = MARKER_NAVIGATOR_NAME;
  marker_navigator.header.frame_id = "/base_link";
  marker_navigator.header.stamp = ros::Time::now();
  marker_navigator.pose = initial_pose_;
  marker_navigator.scale = 1.5;

  InteractiveMarkerControl floorControl;
  floorControl.name = CONTROL_NAVIGATION_NAME;
  floorControl.orientation_mode = InteractiveMarkerControl::INHERIT;
  floorControl.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  floorControl.orientation.x = 0;
  floorControl.orientation.y = 1;
  floorControl.orientation.z = 0;
  floorControl.orientation.w = 1;
  std_msgs::ColorRGBA c;
  c.r = 1.0;
  c.g = 1.0;
  c.b = 0.0;
  c.a = 1.0;
  makeCircle(floorControl, 0.1, 10.0, c);
  marker_navigator.controls.push_back(floorControl);

  server_->insert(marker_driver, boost::bind(&TeleopCOBMarker::processFeedback, this, _1));
  server_->insert(marker_navigator, boost::bind(&TeleopCOBMarker::processFeedback, this, _1));
  server_->applyChanges();
}

}
