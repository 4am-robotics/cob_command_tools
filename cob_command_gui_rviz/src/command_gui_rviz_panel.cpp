#include "cob_command_gui_rviz/command_gui_rviz_panel.h"

#include <wx/event.h>
#include <xmlrpcvalue.h>

namespace rviz
{	
	
	//constructor
	commandguirvizpanel::commandguirvizpanel(wxwindow *parent, const wxstring& title):
	wxpanel(parent, wxid_any, wxdefaultposition, wxdefaultsize, wxtab_traversal, title),
	planning_enabled(false),
	base_diff_enabled(false)
	{		
		//try to load button parameters from ros parameter server
		if(false != getparams())
		{
			//create graphical user interface
			creategui();		
		
			//initialze the action client
			action_client_ = new actionlib::simpleactionclient<cob_script_server::scriptaction>("cob_command_gui_rviz", true);		
		}
		else
		{
			//no parameter available -> error message
			statuslabel_ = new wxstatictext(this, wxid_any, wxt("\n\tstatus: error - parameter does not exist on ros parameter server! \n\n"));	
		}	
	}

	//destructor
	commandguirvizpanel::~commandguirvizpanel() 
	{
	}
	
	//try to load button parameters from ros parameter server
	bool commandguirvizpanel::getparams()
	{	
		std::string param_prefix = "/control_buttons";		
		
		if(false == (ros::param::has(param_prefix)))
		{
			ros_debug_once("parameter '%s' does not exist on ros parameter server, aborting...\n", param_prefix.c_str());			
			return false;			
		}
		
		xmlrpc::xmlrpcvalue group_param;
		ros::param::get(param_prefix, group_param);
				
		xmlrpc::xmlrpcvalue current_group;
		xmlrpc::xmlrpcvalue buttons;
		xmlrpc::xmlrpcvalue group_name;
		xmlrpc::xmlrpcvalue component_name;
		
		std::string groupkey = "groupx";

		//create groups
		for(int i = 0; i < group_param.size(); i++)
		{	
			groupkey[5] = (i+'0'+1);  
			 
			current_group = group_param[groupkey];
			 
			buttons  = current_group["buttons"];
			group_name = current_group["group_name"];
			component_name  = current_group["component_name"];	
			static int id = 101;    		        		    
		 	 
			buttonlist current_buttons;	
					 
			// create buttons in group		 
			for(int j=0; j < buttons.size(); j++)
			{
				if(buttons[j][1] == "move")
				{
					current_buttons.push_back(descripe_button(id, buttons[j], component_name));
				}				
				else if(buttons[j][1] == "move_base_rel")
				{
					current_buttons.push_back(descripe_button(id, buttons[j], component_name));
				}
				else if(buttons[j][1] == "trigger")
				{
					current_buttons.push_back(descripe_button(id, buttons[j], component_name));
					
					if(buttons[j][2] == "stop")
					{
						stop_buttons_.push_back(component_name);
					}
					else if(buttons[j][2] == "init")
					{
						init_buttons_.push_back(component_name);
					}
					else if(buttons[j][2] == "recover")
					{
						recover_buttons_.push_back(component_name);
					}
					
				}
				else if(buttons[j][1] == "stop")
				{
					current_buttons.push_back(descripe_button(id, buttons[j], component_name));
					stop_buttons_.push_back(component_name);
				}
				else if(buttons[j][1] == "mode")
				{
					current_buttons.push_back(descripe_button(id, buttons[j], component_name));
				}
				else 
				{
					std::string fail = buttons[j][1];
					ros_debug_once("function %s not known to rviz_movement_buttons panel", fail.c_str());
				}		
						
				id++;												 
			}	
			
			
			
			//add nav buttons (optional)		
			if("base" == tostlstring(component_name))
			{
				param_prefix = "/nav_buttons";
				
				if(false != (ros::param::has(param_prefix)))
				{
					xmlrpc::xmlrpcvalue nav_buttons_param;
					xmlrpc::xmlrpcvalue nav_buttons;
					
					ros::param::get(param_prefix, nav_buttons_param);
					
					nav_buttons = nav_buttons_param["buttons"];
					
					for(int k=0; k < nav_buttons.size(); k++)
					{
						current_buttons.push_back(descripe_button(id, nav_buttons[k], component_name));	
						id++;
					}
				} 
				else
				{
					ros_debug_once("parameter %s does not exist on ros parameter server, no nav buttons will be available!\n", param_prefix.c_str());		
				}
			}
			
			groups_.push_back(std::pair<std::string, buttonlist>(tostlstring(group_name),current_buttons));
		}				
		
		//uniquify lists
		stop_buttons_.unique();
		init_buttons_.unique();
		recover_buttons_.unique();
		
		return true;
	}	
	
	//creates the graphical user interface
	void commandguirvizpanel::creategui()
	{	
		//main sizer
		wxsizer *mainsizer = new wxboxsizer(wxhorizontal);
		
		//create general box
		sizers_.push_back(createsbgeneral());
		
		//create boxes and buttons inside them
		for(grouplist::const_iterator group_ci = groups_.begin(); group_ci != groups_.end(); group_ci++)
		{
			wxsizer *vsizer = new wxstaticboxsizer(wxvertical, this, towxstring(group_ci->first));
			
			for(buttonlist::const_iterator button_ci = (group_ci->second).begin(); button_ci != (group_ci->second).end(); button_ci++)
			{				
				vsizer->add(addbutton(button_ci->id, towxstring(button_ci->button_name)), 0, wxexpand);		
			}
			
			sizers_.push_back(vsizer);
		}
		
		//add each groupsizer to the mainsizer
		for(unsigned int i = 0; i < sizers_.size(); i++)
		{
			mainsizer->add(sizers_[i], 1, wxexpand);
		}	
		
		mainsizer->setsizehints(this);
		this->setsizerandfit(mainsizer);
	}
	
	//creates the 'general' box with predefined widgets
	wxsizer* commandguirvizpanel::createsbgeneral()
	{			
		//widget ids
		const int genids[5] = {001, 002, 003, 004, 005};		
		
		wxsizer *vsizer = new wxstaticboxsizer(wxvertical, this, wxt("general"));
		
		statuslabel_	= new wxstatictext(this, wxid_any, wxt("\n status: ok"));
		cbplanning_		= new wxcheckbox(this, genids[0], wxt("planning"));
		cbbasediff_		= new wxcheckbox(this, genids[1], wxt("base diff"));
		
		wxbutton *btnstopall	= new wxbutton(this, genids[2], wxt("stop all"), wxdefaultposition, /*wxdefaultsize*/wxsize(30,29), wxbu_exactfit);
		wxbutton *btninitall	= new wxbutton(this, genids[3], wxt("init all"), wxdefaultposition, /*wxdefaultsize*/wxsize(30,29), wxbu_exactfit);
		wxbutton *btnrecoverall	= new wxbutton(this, genids[4], wxt("recover all"), wxdefaultposition, /*wxdefaultsize*/wxsize(30,29), wxbu_exactfit);		
		
		btnstopall->enable(true);
		btninitall->enable(true);	
		btnrecoverall->enable(true);
		
		//connect checkboxes to event handler
		connect(genids[0], wxevt_command_checkbox_clicked, wxcommandeventhandler(commandguirvizpanel::planned_toggle));
		connect(genids[1], wxevt_command_checkbox_clicked, wxcommandeventhandler(commandguirvizpanel::base_mode_toggle));
		
		//connect buttons to event handler
		connect(genids[2], wxevt_command_button_clicked, wxcommandeventhandler(commandguirvizpanel::onstopall));
		connect(genids[3], wxevt_command_button_clicked, wxcommandeventhandler(commandguirvizpanel::oninitall));
		connect(genids[4], wxevt_command_button_clicked, wxcommandeventhandler(commandguirvizpanel::onrecoverall));


		vsizer->add(statuslabel_, 0, wxexpand);
		vsizer->add(btnstopall, 0, wxexpand);
		vsizer->add(btninitall, 0, wxexpand);
		vsizer->add(btnrecoverall, 0, wxexpand);
		
		vsizer->add(cbplanning_, 0, wxexpand);
		vsizer->add(cbbasediff_, 0, wxexpand);	
		
		return vsizer;
	}
 
	//creates a 'button_description'
	button_description commandguirvizpanel::descripe_button(const int &id, xmlrpc::xmlrpcvalue data, const std::string &component_name) const
	{
		button_description btn;
		
		btn.id = id;
		btn.button_name   = tostlstring(data[0]);
		btn.function_name = tostlstring(data[1]);
		btn.args.first    = component_name;
		btn.args.second   = tostlstring(data[2]);
		
	return btn;
	}
	
	//creates a wxbutton and connects it to a event-handler
	wxbutton* commandguirvizpanel::addbutton(const int &id, const wxstring &name)
	{
		wxbutton *but = new wxbutton(this, id, name, wxdefaultposition, wxsize(30,29)/*wxdefaultsize*/, wxbu_exactfit, wxdefaultvalidator, name);
		
		but->enable(true);
		
		connect(id, wxevt_command_button_clicked, wxcommandeventhandler(commandguirvizpanel::onclick));
		 
		return but;
	}

	//event handler for buttons from the config file on ros parameter server
	void commandguirvizpanel::onclick(wxcommandevent& event)
	{			    	
		button_description pressed_button;
		std::string group_name;
		
		pressed_button.id = event.getid();
		
		//use the button_id to get more information about the pressed button		
		for(grouplist::const_iterator group_ci = groups_.begin(); group_ci != groups_.end(); group_ci++)
		{
			for(buttonlist::const_iterator button_ci = group_ci->second.begin(); button_ci != group_ci->second.end(); button_ci++)
			{
				if((button_ci->id == pressed_button.id))
				{
					pressed_button.button_name = button_ci->button_name;
					pressed_button.function_name = button_ci->function_name;
					pressed_button.args = button_ci->args;
					group_name = group_ci->first;
				}
			}
		}
				
		ros_info("'%s-%s' clicked", group_name.c_str(), pressed_button.button_name.c_str());
		
		cob_script_server::scriptgoal goal;
		goal.function_name  = pressed_button.function_name;
		goal.component_name = pressed_button.args.first;
		goal.parameter_name = pressed_button.args.second;
		
		if("arm" == goal.component_name)
		{
			if(false != planning_enabled)
			{
				goal.mode = "planned";
			}
		}
		
		if("base" == goal.component_name)
		{
			if(false != base_diff_enabled)
			{
				goal.mode = "diff";
			}	
		}
		
		//fill in goal here
		action_client_->sendgoal(goal);
		action_client_->waitforresult(ros::duration(1.0));
		if(action_client_->getstate() == actionlib::simpleclientgoalstate::succeeded)
		{
			//set status to 'ok'
			statuslabel_->setlabel(wxt("\n status: ok"));
			
			this->getsizer()->layout();
		}
		else
		{
			//set status to 'no connection'
			statuslabel_->setlabel(wxt("\nstatus: \nno connect. to action server!\n\n"));
			
			this->getsizer()->layout();
			
			ros_debug_once("error! - no connection to action server!\n");
		}
		
		ros_info("current state: %s\n", action_client_->getstate().tostring().c_str());
	}
	
	//event handler for the 'stop all' button
	void commandguirvizpanel::onstopall(wxcommandevent &event)
	{
		cob_script_server::scriptgoal goal;
		bool success = true; 
		
		goal.function_name  = "stop";
		
		ros_info("'stop all' clicked");
		
		for(stringlist::const_iterator ci = stop_buttons_.begin(); ci != stop_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
			
			action_client_->sendgoal(goal);
			action_client_->waitforresult(ros::duration(1.0));
		    
			if("succeeded" != action_client_->getstate().tostring())
			{
				success = false;
				ros_info("warning: component %s not properly stopped\n", goal.component_name.c_str());
			}
			
			ros_info("current state: %s\n", action_client_->getstate().tostring().c_str());
		}
		
		if(false != success)
		{ 
			ros_info("all components stopped\n");
			
			statuslabel_->setlabel(wxt("\n status: ok"));
			this->getsizer()->layout();
		}
		else
		{
			//set status to 'no connection'
			statuslabel_->setlabel(wxt("\nstatus: \nno connect. to action server!\n\n"));
			
			this->getsizer()->layout();
			ros_debug_once("error! - no connection to action server!\n");
		}
	}
	
	//event handler for the 'init all' button
	void commandguirvizpanel::oninitall(wxcommandevent &event)
	{
		cob_script_server::scriptgoal goal;
		bool success = true; 
		
		goal.function_name = "init";
		
		ros_info("'init all' clicked");
		
		for(stringlist::const_iterator ci = init_buttons_.begin(); ci != init_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
			
			action_client_->sendgoal(goal);
			action_client_->waitforresult(ros::duration(1.0));
		    
			if("succeeded" != action_client_->getstate().tostring())
			{
				success = false;
				ros_info("warning: component %s not properly initialized\n", goal.component_name.c_str());
			}
			
			ros_info("current state: %s\n", action_client_->getstate().tostring().c_str());
		}
		
		if(false != success)
		{ 
			statuslabel_->setlabel(wxt("\n status: ok"));
			this->getsizer()->layout();
			
			ros_info("all components initialized\n");
		}
		else
		{
			//set status to 'no connection'
			statuslabel_->setlabel(wxt("\nstatus: \nno connect. to action server!\n\n"));
			this->getsizer()->layout();
			
			ros_debug_once("error! - no connection to action server!\n");
		}
	}
	
	//event handler for the 'recover all' button
	void commandguirvizpanel::onrecoverall(wxcommandevent &event)
	{
		cob_script_server::scriptgoal goal;
		bool success = true; 
		
		goal.function_name = "recover";
		
		ros_info("'recover all' clicked");
		
		for(stringlist::const_iterator ci = recover_buttons_.begin(); ci != recover_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
			
			action_client_->sendgoal(goal);
			action_client_->waitforresult(ros::duration(1.0));
			
			if("succeeded" != action_client_->getstate().tostring())
		    {
				success = false;
				ros_info("warning: component %s not properly recovered\n", goal.component_name.c_str());
			}
			
			ros_info("current state: %s\n", action_client_->getstate().tostring().c_str());
		}
		
		if(false != success)
		{ 
			statuslabel_->setlabel(wxt("\n status: ok"));
			this->getsizer()->layout();
			
			ros_info("all components recovered\n");
		}
		else
		{
			//set status to 'no connection'
			statuslabel_->setlabel(wxt("\nstatus: \nno connect. to action server!\n\n"));
			this->getsizer()->layout();
			
			ros_debug_once("error! - no connection to action server!\n");
		}
	}
	
	//event handler for the 'planning' checkbox
	inline void commandguirvizpanel::planned_toggle(wxcommandevent &event)
	{
		planning_enabled = !planning_enabled;
	}
	
	//event handler for the 'base diff' checkbox
	inline void commandguirvizpanel::base_mode_toggle(wxcommandevent &event)
	{
		base_diff_enabled = !base_diff_enabled;
	}

	//converts a std::string into a wxstring
	inline wxstring commandguirvizpanel::towxstring(const std::string &temp) const
	{
		return wxstring(temp.c_str(), wxconvutf8);
	}
	
	//coverts a xmlrpcvalue of 'typestring' or a wxstring into std::string
	inline std::string commandguirvizpanel::tostlstring(xmlrpc::xmlrpcvalue temp, wxstring wxtemp) const
	{	
		if(false == wxtemp.isempty())
		{
			return std::string(wxtemp.mb_str());
		}
		else
		{
			return static_cast<std::string>(temp);
		}
	}

}
