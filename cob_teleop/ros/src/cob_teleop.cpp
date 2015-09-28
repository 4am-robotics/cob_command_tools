/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_command_tools
 * \note
 *   ROS package name: cob_teleop
 *
 * \author
 *   Author: Nadia Hammoudeh García, email:nadia.hammoudeh.garcia@ipa.fhg.de
 * \author
 *   Supervised by: Nadia Hammoudeh García, email:nadia.hammoudeh.garcia@ipa.fhg.de
 *
 * \date Date of creation: August 2015
 *
 * \brief
 *   Implementation of teleoperation node.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <cob_light/SetLightModeAction.h>
#include <cob_script_server/ScriptAction.h>
#include <cob_sound/SayAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Trigger.h>
#include <XmlRpcValue.h>

const int PUBLISH_FREQ = 30.0;

class CobTeleop
{
public:

  struct component_config{
    std::string key;
    std::string twist_topic_name;
    std::string vel_group_topic_name;
    std::string sss_default_target;
    std::vector<double> joint_velocity;
    ros::Publisher vel_group_controller_publisher_;
    ros::Publisher twist_controller_publisher_;
    std::vector<double> twist_max_vel; //max_vx_,max_vy_,max_vz_,max_rotx_,max_roty_,max_rotz_
    std::vector<double> twist_max_acc; //max_ax_,max_ay_,max_arotz_
  };

  std::map<std::string,component_config> component_config_;

	//axis
  int axis_vx_,axis_vy_,axis_vz_,axis_roll_,axis_pitch_,axis_yaw_;

	//buttons
	//mode 1: Base
  int run_button_;
	//mode 2: Trajectory controller (to default target position using sss.move)
	//mode 3: Velocity group controller
  int right_indicator_button_;
  int left_indicator_button_;
  int up_down_button_;
  int right_left_button_;
	//mode 4: Twist controller

	//common
  int deadman_button_;
  int safety_button_;
  int init_button_;
  bool joy_active_;
  double run_factor_, run_factor_param_;
  int joy_num_modes_;
  int mode_switch_button_;
  int mode_;
  XmlRpc::XmlRpcValue LEDS_;
  XmlRpc::XmlRpcValue led_mode_;
  sensor_msgs::JoyFeedbackArray joyfb;

  XmlRpc::XmlRpcValue components_;
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;  //subscribe topic joy
  ros::Subscriber joint_states_sub_;  //subscribe topic joint_states

  typedef actionlib::SimpleActionClient<cob_script_server::ScriptAction> Client_;
  cob_script_server::ScriptGoal sss_;
  Client_ * sss_client_;
  typedef actionlib::SimpleActionClient<cob_sound::SayAction> SayClient_;
  SayClient_ * sayclient_;
  typedef actionlib::SimpleActionClient<cob_light::SetLightModeAction> SetLightClient_;
  SetLightClient_ * setlightclient_;
  cob_light::SetLightModeGoal lightgoal;
  

  void getConfigurationFromParameters();
  void init();
  void updateBase();
  void say(std::string text, bool blocking);
  void joy_cb(const sensor_msgs::Joy::ConstPtr &joy_msg);
  sensor_msgs::JoyFeedbackArray switch_mode();
  std::vector<double> vel_old_;
  std::vector<double> vel_req_;
  std::vector<double> vel_base_;
};

void CobTeleop::getConfigurationFromParameters()
{
  if(n_.hasParam("components"))
  {
    ROS_DEBUG("components found ");
    n_.getParam("components", components_);
    if(components_.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_DEBUG("components are of type struct with size %d",(int)components_.size());
      for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=components_.begin();p!=components_.end();++p)
      {
        std::string comp_name = p->first;
        ROS_DEBUG("component name: %s",comp_name.c_str());
        XmlRpc::XmlRpcValue comp_struc = p->second;
        if(comp_struc.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          ROS_WARN("invalid component, name: %s",comp_name.c_str());
        component_config tempComponent;
        for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator ps=comp_struc.begin();ps!=comp_struc.end();++ps)
        {
          std::string par_name = ps->first;
          ROS_DEBUG("par name: %s",par_name.c_str());
          if(par_name.compare("twist_topic_name")==0)
          {
            ROS_DEBUG("twist topic name found");
            XmlRpc::XmlRpcValue twist_topic_name = ps->second;
            ROS_ASSERT(twist_topic_name.getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string s((std::string)twist_topic_name);
            ROS_DEBUG("twist_topic_name found = %s",s.c_str());
            tempComponent.twist_topic_name = s;
            tempComponent.twist_controller_publisher_ = n_.advertise<geometry_msgs::Twist>((s),1);
          }
          else if(par_name.compare("velocity_topic_name")==0)
          {
            ROS_DEBUG("topic name found");
            XmlRpc::XmlRpcValue vel_group_topic_name = ps->second;
            ROS_ASSERT(vel_group_topic_name.getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string s((std::string)vel_group_topic_name);
            ROS_DEBUG("topic_name found = %s",s.c_str());
            tempComponent.vel_group_topic_name = s;
            tempComponent.vel_group_controller_publisher_ = n_.advertise<std_msgs::Float64MultiArray>((s),1);
          }
          else if(par_name.compare("sss_default_target")==0)
          {
            ROS_DEBUG("default target position found");
            XmlRpc::XmlRpcValue sss_default_target = ps->second;
            ROS_ASSERT(sss_default_target.getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string s((std::string)sss_default_target);
            ROS_DEBUG("sss_default_target found = %s",s.c_str());
            tempComponent.sss_default_target = s;
          }
          else if(par_name.compare("joint_velocity")==0){
            ROS_DEBUG("joint vels found");
            XmlRpc::XmlRpcValue joint_velocity = ps->second;
            ROS_ASSERT(joint_velocity.getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_DEBUG("joint_velocity.size: %d \n", joint_velocity.size());
            for(int i=0;i<joint_velocity.size();i++)
            {
              ROS_ASSERT(joint_velocity[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
              double vel((double)joint_velocity[i]);
              ROS_DEBUG("joint_velocity found = %f",vel);
              tempComponent.joint_velocity.push_back(vel);
            }
          }else if(par_name.compare("twist_max_velocity")==0){
            ROS_DEBUG("max Velocity found");
            XmlRpc::XmlRpcValue twist_max_velocity = ps->second;
            ROS_ASSERT(twist_max_velocity.getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_DEBUG("twist_max_velocity.size: %d \n", twist_max_velocity.size());
            for(int i=0;i<twist_max_velocity.size();i++)
            {
              ROS_ASSERT(twist_max_velocity[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
              double vel((double)twist_max_velocity[i]);
              ROS_DEBUG("twist_max_velocity found = %f",vel);
              tempComponent.twist_max_vel.push_back(vel);
              }
            }else if(par_name.compare("twist_max_acc")==0){
            ROS_DEBUG("max Velocity found");
            XmlRpc::XmlRpcValue twist_max_acc = ps->second;
            ROS_ASSERT(twist_max_acc.getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_DEBUG("twist_max_acc.size: %d \n", twist_max_acc.size());
            for(int i=0;i<twist_max_acc.size();i++)
            {
              ROS_ASSERT(twist_max_acc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
              double vel((double)twist_max_acc[i]);
              ROS_DEBUG("twist_max_acc found = %f",vel);
              tempComponent.twist_max_acc.push_back(vel);
            }
          }
        }
        ROS_DEBUG("%s module stored",comp_name.c_str());
        component_config_.insert(std::pair<std::string,component_config>(comp_name,tempComponent));
      }
    }
  }
  std::string light_action;
  n_.getParam( "light_action_name", light_action);
  setlightclient_ = new SetLightClient_(light_action, true);
  
  sayclient_ = new SayClient_("/sound/say", true);
  sss_client_ = new Client_("/script_server", true);

  vel_req_.resize(component_config_["base"].twist_max_acc.size());
  vel_old_.resize(component_config_["base"].twist_max_acc.size());
  vel_base_.resize(component_config_["base"].twist_max_acc.size());
  for(unsigned int i=0; i<component_config_["base"].twist_max_acc.size(); i++){
    vel_old_[i]=0;
    vel_req_[i]=0;
  }
}
sensor_msgs::JoyFeedbackArray CobTeleop::switch_mode(){
  ++mode_;

  if (mode_ > joy_num_modes_)
  {
    mode_ = 1;
  }

  std::string saytext;
  cob_light::LightMode light;
  light.timeout = 1;

  if (mode_ == 1){
    ROS_INFO("Switched to mode 1: move the base using twist controller");
     saytext = "Base mode";
     light.pulses = 1;
  }if (mode_ == 2){
    ROS_INFO("Switched to mode 2: move the actuators to a default position (Trajectory controller)");
     saytext = "Default position mode";
     light.pulses = 2;
  }if(mode_ == 3){
    ROS_INFO("Switched to mode 3: move the actuators using joint group velocity controller");
     saytext = "Velocity mode";
     light.pulses = 3;
  }if(mode_ == 4){
    ROS_INFO("Switched to mode 4: move the actuators in cartesian mode using twist controller");
     saytext = "Cartesian mode";
     light.pulses = 4;
  }

  light.mode = 2;
  light.frequency = 5;
  std_msgs::ColorRGBA color;
  color.r = 0;
  color.g = 1;
  color.b = 0;
  color.a = 1;
  light.pulses = mode_;
  light.color = color;
  lightgoal.mode = light;
//  setlightclient_->sendGoal(lightgoal);
  say(saytext, false);

  LEDS_=led_mode_[mode_];

  for (int i=0; i<4; i++)
  {
      joyfb.array.resize(4);
      joyfb.array[i].type=0;
      joyfb.array[i].id=i;
      joyfb.array[i].intensity=static_cast<int>(LEDS_[i]);
  }
  return joyfb;
}

void CobTeleop::updateBase(){
  if (joy_active_){
    if(mode_==1){
      double dt = 1.0/double(PUBLISH_FREQ);
      geometry_msgs::Twist base_cmd;
      if(!joy_active_){
        for(unsigned int i=0; i<3; i++){
          vel_old_[i]=0;
          vel_req_[i]=0;
        }
      }

      for( int i =0; i<3; i++)
      {
        // filter v with ramp
        if ((vel_req_[i]-vel_old_[i])/dt > component_config_["base"].twist_max_acc[i])
        {
          vel_base_[i] = vel_old_[i] + component_config_["base"].twist_max_acc[i]*dt;
        }
        else if((vel_req_[i]-vel_old_[i])/dt < -component_config_["base"].twist_max_acc[i])
        {
          vel_base_[i] = vel_old_[i] - component_config_["base"].twist_max_acc[i]*dt;
        }
        else
        {
          vel_base_[i] = vel_req_[i];
        }
        vel_old_[i] = vel_base_[i];

      base_cmd.linear.x = vel_base_[0];
      base_cmd.linear.y = vel_base_[1];
      base_cmd.angular.z = vel_base_[2];
      component_config_["base"].twist_controller_publisher_.publish(base_cmd);
      }
    }
  }
}

void CobTeleop::say(std::string text, bool blocking)
{
  cob_sound::SayGoal saygoal;
  std::replace(text.begin(), text.end(), '_', ' ');
  saygoal.text = text;
  sayclient_->sendGoal(saygoal);
  if (blocking)
  {
    sayclient_->waitForResult(ros::Duration(5));
  }
}

void CobTeleop::joy_cb(const sensor_msgs::Joy::ConstPtr &joy_msg){

  if(deadman_button_>=0 && deadman_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[deadman_button_]==1)
  {
    ROS_INFO("joystick is active");
  }else
  {
    for(unsigned int i=0; i<component_config_["base"].twist_max_acc.size(); i++){
		    vel_req_[i]=0;
		    vel_old_[i]=0;
	    }
    ROS_DEBUG("joystick is not active");
    joy_active_ = false;
    return;
  }

  if(mode_switch_button_>=0 && mode_switch_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[mode_switch_button_]==1)
  {
    ROS_INFO("Switch mode button pressed");
    switch_mode();
  }
  
  if(run_button_>=0 && run_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[run_button_]==1)
  {
    run_factor_ = run_factor_param_;
  }
  else //button release
  {
    run_factor_ = 1.0;
  }

  if(!sss_client_ -> waitForServer(ros::Duration(3.0))){
    ROS_INFO("Waiting for script server");
    return;
  }

  if(init_button_>=0 && init_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[init_button_]==1)
  {
    ROS_INFO("Init and recover issued");
    say("init and recover issued", true);

    for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=components_.begin();p!=components_.end();++p)
    {
      std::string comp_name = p->first;
      say(comp_name, true);

      ROS_INFO("Init %s",comp_name.c_str());
      sss_.component_name = comp_name.c_str();
      sss_.function_name="init";
      sss_client_->sendGoal(sss_);
      sss_client_ -> waitForResult();
      if(sss_client_->getResult()->error_code != 0)
      {
        say("Init " + comp_name + " failed", false);
      }

      ROS_INFO("Recover %s",comp_name.c_str());
      sss_.function_name="recover";
      sss_client_->sendGoal(sss_);
      sss_client_ -> waitForResult();
      if(sss_client_->getResult()->error_code != 0)
      {
        say("Recover " + comp_name + " failed", false);
      }
    }
  }

//-------MODE 1
  if (mode_==1){
    ROS_DEBUG("Mode 1: Move the base using twist controller");
    if(axis_vx_>=0 && axis_vx_<(int)joy_msg->axes.size()){
      joy_active_ = true;
      vel_req_[0] = joy_msg->axes[axis_vx_]*component_config_["base"].twist_max_vel[0]*run_factor_;
    }else{
      vel_req_[0] =0.0;
    }if(axis_vy_>=0 && axis_vy_<(int)joy_msg->axes.size()){
      joy_active_ = true;
      vel_req_[1] = joy_msg->axes[axis_vy_]*component_config_["base"].twist_max_vel[1]*run_factor_;
    }else{
      vel_req_[1] = 0.0;
    }if(axis_yaw_>=0 && axis_yaw_<(int)joy_msg->axes.size()){
      joy_active_ = true;
      vel_req_[2] = joy_msg->axes[axis_yaw_]*component_config_["base"].twist_max_vel[2]*run_factor_;
    }else{
      vel_req_[2] = 0.0;
    }
  }
//-------MODE 2
  if (mode_==2){
    ROS_DEBUG("Mode 2: Move the actuators to a default position (Trajectory controller)");

    for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=components_.begin();p!=components_.end();++p)
    {
      int component_sss_default_target_button_temp = -1;
      std::string comp_name = p->first;
      std::string component_sss_default_target_button = comp_name + "_sss_default_target_button";
      n_.getParam(component_sss_default_target_button,component_sss_default_target_button_temp);
      if (component_sss_default_target_button_temp == -1){
        ROS_DEBUG("%s_sss_default_target_button parameter not defined",comp_name.c_str());
      }
      if(component_sss_default_target_button_temp>=0 && component_sss_default_target_button_temp<(int)joy_msg->buttons.size() && joy_msg->buttons[component_sss_default_target_button_temp]==1){
        sss_.component_name = comp_name;
        sss_.function_name = "move";
        sss_.parameter_name = component_config_[comp_name].sss_default_target.c_str();
        sss_client_->sendGoal(sss_);
        ROS_INFO("Move %s to %s",comp_name.c_str(), component_config_[comp_name].sss_default_target.c_str());
      }
    }
  }
//-------MODE 3
   if (mode_==3){
     ROS_DEBUG("Mode 3: Move the actuators using the group velocity controller");
      for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=components_.begin();p!=components_.end();++p)
      {
        int count = 0;
        int component_joint_button_temp = -1;
        std::string comp_name = p->first;
        int size = ceil((component_config_[comp_name].joint_velocity.size()+1)/2);
        for(int i=0; i < size; ++i){
          std::ostringstream component_joint_button_stream;
          component_joint_button_stream << comp_name << "_joint" << i+1 << "_button";
          std::string component_joint_button = component_joint_button_stream.str();
          n_.getParam(component_joint_button,component_joint_button_temp);
          std_msgs::Float64MultiArray vel_cmd;
          count++;
          if(!(comp_name.find("left") != std::string::npos) && !(comp_name.find("right") != std::string::npos)){
            if(component_joint_button_temp>=0 && component_joint_button_temp<(int)joy_msg->buttons.size() && joy_msg->buttons[component_joint_button_temp]==1 && joy_msg->buttons[right_indicator_button_]==0 && joy_msg->buttons[left_indicator_button_]==0){
              joy_active_ = true;
              ROS_INFO("%s velocity mode",comp_name.c_str());
              vel_cmd.data.resize(component_config_[comp_name].joint_velocity.size());
              if(up_down_button_>=0 && up_down_button_<(int)joy_msg->axes.size()){
                vel_cmd.data[count+i-1]=joy_msg->axes[up_down_button_]*component_config_[comp_name].joint_velocity[count+i-1];
              }
              if((i+1)*2 <= component_config_[comp_name].joint_velocity.size()){
                if(right_left_button_>=0 && right_left_button_<(int)joy_msg->axes.size()){
                  vel_cmd.data[count+i]=joy_msg->axes[right_left_button_]*component_config_[comp_name].joint_velocity[count+i];
                }
              }
            component_config_[comp_name].vel_group_controller_publisher_.publish(vel_cmd);
          }
        }
        else if(!(comp_name.find("left") != std::string::npos) && (comp_name.find("right") != std::string::npos)){
          if(component_joint_button_temp>=0 && component_joint_button_temp<(int)joy_msg->buttons.size() && joy_msg->buttons[component_joint_button_temp]==1 && joy_msg->buttons[right_indicator_button_]==1){
            joy_active_ = true;
            ROS_INFO("%s velocity mode",comp_name.c_str());
            vel_cmd.data.resize(component_config_[comp_name].joint_velocity.size());
            if(up_down_button_>=0 && up_down_button_<(int)joy_msg->axes.size()){
              vel_cmd.data[count+i-1]=joy_msg->axes[up_down_button_]*component_config_[comp_name].joint_velocity[count+i-1];
            }
            if((i+1)*2 <= component_config_[comp_name].joint_velocity.size()){
              if(right_left_button_>=0 && right_left_button_<(int)joy_msg->axes.size()){
                vel_cmd.data[count+i]=joy_msg->axes[right_left_button_]*component_config_[comp_name].joint_velocity[count+i];
              }
            }
            component_config_[comp_name].vel_group_controller_publisher_.publish(vel_cmd);
          }
        }
        else if((comp_name.find("left") != std::string::npos) && !(comp_name.find("right") != std::string::npos)){
          if(component_joint_button_temp>=0 && component_joint_button_temp<(int)joy_msg->buttons.size() && joy_msg->buttons[component_joint_button_temp]==1 && joy_msg->buttons[left_indicator_button_]==1){
            joy_active_ = true;
            ROS_INFO("%s velocity mode",comp_name.c_str());
            vel_cmd.data.resize(component_config_[comp_name].joint_velocity.size());
            if(up_down_button_>=0 && up_down_button_<(int)joy_msg->axes.size()){
              vel_cmd.data[count+i-1]=joy_msg->axes[up_down_button_]*component_config_[comp_name].joint_velocity[count+i-1];
            }
            if((i+1)*2 <= component_config_[comp_name].joint_velocity.size()){
              if(right_left_button_>=0 && right_left_button_<(int)joy_msg->axes.size()){
                vel_cmd.data[count+i]=joy_msg->axes[right_left_button_]*component_config_[comp_name].joint_velocity[count+i];
              }
            }
            component_config_[comp_name].vel_group_controller_publisher_.publish(vel_cmd);
          }
        }
      }
    }
  }

//-------MODE 4
  if (mode_==4){
    ROS_DEBUG("Mode 4: Move the actuators using the twist controller");
    for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=components_.begin();p!=components_.end();++p)
    {
      int component_twist_button_temp = -1;
      std::string comp_name = p->first;
      std::string component_twist_button = comp_name + "_twist_button";
      n_.getParam(component_twist_button,component_twist_button_temp);
      geometry_msgs::Twist twist_cmd;
      if (component_twist_button_temp>=0 && component_twist_button_temp<(int)joy_msg->buttons.size() && joy_msg->buttons[component_twist_button_temp]==1){
        joy_active_ = true;
        ROS_INFO("%s twist mode",comp_name.c_str());
        if(axis_vx_>=0 && axis_vx_<(int)joy_msg->axes.size())
          twist_cmd.linear.x = joy_msg->axes[axis_vx_]*component_config_[comp_name].twist_max_vel[0]; //*run_factor_;
        else
          twist_cmd.linear.x =0.0;
        if(axis_vy_>=0 && axis_vy_<(int)joy_msg->axes.size())
          twist_cmd.linear.y = joy_msg->axes[axis_vy_]*component_config_[comp_name].twist_max_vel[1]; //*run_factor_;
        else
          twist_cmd.linear.y =0.0;
        if(axis_vz_>=0 && axis_vz_<(int)joy_msg->axes.size())
          twist_cmd.linear.z = joy_msg->axes[axis_vz_]*component_config_[comp_name].twist_max_vel[2]; //*run_factor_;
        else
          twist_cmd.linear.z =0.0;
        if(axis_roll_>=0 && axis_roll_<(int)joy_msg->axes.size())
          twist_cmd.angular.x = joy_msg->axes[axis_roll_]*component_config_[comp_name].twist_max_vel[3]; //*run_factor_;
        else
          twist_cmd.angular.x =0.0;
        if(axis_pitch_>=0 && axis_pitch_<(int)joy_msg->axes.size())
          twist_cmd.angular.y = joy_msg->axes[axis_pitch_]*component_config_[comp_name].twist_max_vel[4]; //*run_factor_;
        else
          twist_cmd.angular.y =0.0;
        if(axis_yaw_>=0 && axis_yaw_<(int)joy_msg->axes.size())
          twist_cmd.angular.z = joy_msg->axes[axis_yaw_]*component_config_[comp_name].twist_max_vel[5]; //*run_factor_;
        else
          twist_cmd.angular.z =0.0;
        component_config_[comp_name].twist_controller_publisher_.publish(twist_cmd);
      }
    }
  }
}

/*!
 * \brief Initializes node to get parameters, subscribe to topics.
 */
void CobTeleop::init()
{

  n_ = ros::NodeHandle("~");
  if(!n_.hasParam("components")){
    ROS_ERROR("parameter components does not exist on ROS Parameter Server, aborting...");
    exit(0);
	}
	// common
  n_.param("run_factor",run_factor_param_,1.5);

	// joy config
  n_.param("joy_num_modes",joy_num_modes_,2);
  n_.param("mode_switch_button",mode_switch_button_,0);

	// assign axis
  n_.param("axis_vx",axis_vx_,17);
  n_.param("axis_vy",axis_vy_,16);
  n_.param("axis_vz",axis_vz_,17);
  n_.param("axis_roll",axis_roll_,16);
  n_.param("axis_pitch",axis_pitch_,19);
  n_.param("axis_yaw",axis_yaw_,19);

	// assign buttons
  n_.param("deadman_button",deadman_button_,11);
  n_.param("safety_button",safety_button_,10);
  n_.param("init_button",init_button_,3);

  n_.param("run_button",run_button_,9);

  n_.param("right_indicator_button",right_indicator_button_,9);
  n_.param("left_indicator_button",left_indicator_button_,8);
  n_.param("up_down_button",up_down_button_,4);
  n_.param("right_left_button",right_left_button_,5);

	// output for debugging
  ROS_DEBUG("init::axis_vx: %d",axis_vx_);
  ROS_DEBUG("init::axis_vy: %d",axis_vy_);
  ROS_DEBUG("init::axis_vz: %d",axis_vz_);
  ROS_DEBUG("init::axis_roll: %d",axis_roll_);
  ROS_DEBUG("init::axis_pitch: %d",axis_pitch_);
  ROS_DEBUG("init::axis_yaw: %d",axis_yaw_);

  ROS_DEBUG("init::deadman_button: %d",deadman_button_);
  ROS_DEBUG("init::safety_button: %d",safety_button_);
  ROS_DEBUG("init::init_button: %d",init_button_);
  ROS_DEBUG("init::run_button: %d",run_button_);

  ROS_DEBUG("init::right_indicator_button: %d",right_indicator_button_);
  ROS_DEBUG("init::left_indicator_button: %d",left_indicator_button_);
  ROS_DEBUG("init::up_down_button: %d",up_down_button_);
  ROS_DEBUG("init::right_left_button: %d",right_left_button_);

  joy_sub_ = n_.subscribe("/joy",1,&CobTeleop::joy_cb,this);
  mode_ = 1;
  LEDS_=led_mode_[mode_];
  joy_active_ = false;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_teleop");
  CobTeleop  cob_teleop;
  cob_teleop.init();
  cob_teleop.getConfigurationFromParameters();
  ros::Rate loop_rate(PUBLISH_FREQ); //Hz
  while(cob_teleop.n_.ok())
  {
    cob_teleop.updateBase();
    ros::spinOnce();
    loop_rate.sleep();
  }

  exit(0);
  return(0);
}


