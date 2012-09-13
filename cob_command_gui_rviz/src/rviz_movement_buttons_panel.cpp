#include "cob_rviz_movement_control/rviz_movement_buttons_panel.h"

#include <wx/event.h>
#include <wx/filedlg.h>
#include <pthread.h>
#include <XmlRpcValue.h>

namespace rviz
{	
	
	//constructor
	RvizMovementButtonsPanel::RvizMovementButtonsPanel(wxWindow *parent, const wxString& title):
	wxPanel(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, title),
	planning_enabled(false),
	base_diff_enabled(false)
	{		
		
		//try to load button parameters from ROS Parameter Server
		if(false != Getparams())
		{
			//create graphical user interface
			Creategui();		
		
			//initialze the action client
			action_client_ = new actionlib::SimpleActionClient<cob_script_server::ScriptAction>("cob_rviz_movement_control", true);		
		}
		else
		{
			//no parameter available -> launch startup file
		    system("roslaunch cob_rviz_movement_control startup.launch");
			
			//check again
			if(false != Getparams())
			{
				//create graphical user interface
				Creategui();		
		
				//initialze the action client
				action_client_ = new actionlib::SimpleActionClient<cob_script_server::ScriptAction>("cob_rviz_movement_control", true);	
			}
			else
			{
				//still no parameter available -> Error message
				statuslabel_ = new wxStaticText(this, wxID_ANY, wxT("\n\tStatus: Error - Parameter does not exist on ROS Parameter Server! \n\n"));
				statuslabel_->SetForegroundColour(wxColor(255,0,0));		
			}
		}	
		
    }

	//destructor
	RvizMovementButtonsPanel::~RvizMovementButtonsPanel() 
	{
	}
	
	//loads button parameters from ROS Parameter Server
	bool RvizMovementButtonsPanel::Getparams()
	{
		//counts the failed parameter loading attempts
		static int failed_attempts = 0;
		
		//load parameter from ROS Parameter Server
		std::string param_prefix = "/control_buttons";		
		
		if(false == (ros::param::has(param_prefix)))
		{
			if(2 == ++failed_attempts)
			{
				ROS_DEBUG_ONCE("Parameter '%s' does not exist on ROS Parameter Server, aborting...", param_prefix.c_str());		
				ROS_INFO("Parameter '%s' does not exist on ROS Parameter Server, aborting...", param_prefix.c_str());		
			}
			return false;			
		}
		
		XmlRpc::XmlRpcValue group_param;
        ros::param::get(param_prefix, group_param);
				
		XmlRpc::XmlRpcValue current_group;
		XmlRpc::XmlRpcValue buttons;
		XmlRpc::XmlRpcValue group_name;
		XmlRpc::XmlRpcValue component_name;
		
		std::string groupkey = "groupx";

		//create groups
		for(int i = 0; i < group_param.size(); i++)
		{	
			groupkey[5] = (i+'0'+1);  
			 
			current_group = group_param[groupkey];
			 
		    buttons  = current_group["buttons"];
	        group_name = current_group["group_name"];
		    component_name  = current_group["component_name"];	
		    static int id = 201;    		        		    
		 	 
			Buttonlist current_buttons;	
			
			//std::cout << "\n group_name: " << group_name;
			//std::cout << "\n component_name: " << component_name;	 
					 
			// create buttons in group		 
		    for(int j=0; j < buttons.size(); j++)
		    {
				/*  Output: group_name
				 * 		group_name
				 * 		component_name
				 * 		button_name, function_name, parameter_name, id
				 */ 
				//std::cout <<"\n\t" << buttons[j][0] << "," << buttons[j][1] << "," <<  buttons[j][2]  << "," << id << "\n\n";		
			
				/*
				 * hier spar ich mir einen funktionsaufruf wenn ich die konstante rechts schreibe,
				 * sonst wäre TostlString() notwendig
				*/	
				if(buttons[j][1] == "move")
				{
					current_buttons.push_back(Descripe_button(id, buttons[j], component_name));
				}				
				else if(buttons[j][1] == "move_base_rel")
				{
					current_buttons.push_back(Descripe_button(id, buttons[j], component_name));
				}
				else if(buttons[j][1] == "trigger")
				{
					current_buttons.push_back(Descripe_button(id, buttons[j], component_name));
					
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
					current_buttons.push_back(Descripe_button(id, buttons[j], component_name));
					stop_buttons_.push_back(component_name);
				}
				else if(buttons[j][1] == "mode")
				{
					current_buttons.push_back(Descripe_button(id, buttons[j], component_name));
				}
				else 
				{
					std::string fail = buttons[j][1];
					ROS_DEBUG_ONCE("Function %s not known to rviz_movement_buttons panel", fail.c_str());
				}		
						
				id++;												 
			}	
			
			
			
			//add nav buttons (optional)		
			if("base" == TostlString(component_name))
			{
				param_prefix = "/nav_buttons";
				
				if(false != (ros::param::has(param_prefix)))
				{
					XmlRpc::XmlRpcValue nav_buttons_param;
					XmlRpc::XmlRpcValue nav_buttons;
					
					ros::param::get(param_prefix, nav_buttons_param);
					
					nav_buttons = nav_buttons_param["buttons"];
					
					for(int k=0; k < nav_buttons.size(); k++)
					{
						current_buttons.push_back(Descripe_button(id, nav_buttons[k], component_name));	
						id++;
					}
				} 
				else
				{
					ROS_DEBUG_ONCE("parameter %s does not exist on ROS Parameter Server, no nav buttons will be available.", param_prefix.c_str());		
				}
			}
				
			groups_[TostlString(group_name)] = current_buttons;
		}				
		
		stop_buttons_.unique();
		init_buttons_.unique();
		recover_buttons_.unique();
		
		return true;
	}	
	
	//creates the graphical user interface
	void RvizMovementButtonsPanel::Creategui()
	{	
		//main sizer
		wxSizer *mainsizer = new wxBoxSizer(wxHORIZONTAL);
		
		//wxSizer *tempsizer = new wxBoxSizer(wxVERTICAL);
		
		//tempsizer->Add(createsbGeneral(), 1, wxEXPAND | wxBOTTOM, 10);
		//tempsizer->Add(createsbSMACH(), 1, wxEXPAND | wxTOP, 10);
		
		//sizers_.push_back(tempsizer);
		
		//create general box
		sizers_.push_back(CreatesbGeneral());
		
		//create boxes and buttons inside them
		for(Groupdict::const_iterator group_ci = groups_.begin(); group_ci != groups_.end(); group_ci++)
		{
			wxSizer *vsizer = new wxStaticBoxSizer(wxVERTICAL, this, TowxString(group_ci->first));
			
			for(Buttonlist::const_iterator button_ci = (group_ci->second).begin(); button_ci != (group_ci->second).end(); button_ci++)
			{				
				vsizer->Add(AddButton(button_ci->id, TowxString(button_ci->button_name)), 0, wxEXPAND);		
			}
			
			sizers_.push_back(vsizer);
		}
		
		//add staticboxsizer to mainsizer
	  	for(unsigned int i = 0; i < sizers_.size(); i++)
	    {
			mainsizer->Add(sizers_[i], 1, wxEXPAND);
		}	
		
		mainsizer->SetSizeHints(this);
		this->SetSizerAndFit(mainsizer);
	}
	
	//creates the 'general' box with predefined widgets
	wxSizer* RvizMovementButtonsPanel::CreatesbGeneral()
	{			
		//Widget IDs
		const int GENIDs[7] = {001, 002, 003, 004, 005, 006, 007};		
		
		wxSizer *vsizer = new wxStaticBoxSizer(wxVERTICAL, this, wxT("General"));
		
		statuslabel_            = new wxStaticText(this, wxID_ANY, wxT("\n Status: OK"));
		cbPlanning_		        = new wxCheckBox(this, GENIDs[0], wxT("Planning"));
		cbBaseDiff_		  	    = new wxCheckBox(this, GENIDs[1], wxT("Base Diff"));
		
		wxButton *btnStopAll    = new wxButton(this, GENIDs[2], wxT("stop all"), wxDefaultPosition, /*wxDefaultSize*/wxSize(30,29), wxBU_EXACTFIT);
		wxButton *btnInitAll    = new wxButton(this, GENIDs[3], wxT("init all"), wxDefaultPosition, /*wxDefaultSize*/wxSize(30,29), wxBU_EXACTFIT);
		wxButton *btnRecoverAll = new wxButton(this, GENIDs[4], wxT("recover all"), wxDefaultPosition, /*wxDefaultSize*/wxSize(30,29), wxBU_EXACTFIT);		
		
		btnStopAll->Enable(true);
		btnInitAll->Enable(true);	
		btnRecoverAll->Enable(true);
		
		//connect checkboxes to event handler
		Connect(GENIDs[0], wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::Planned_toggle));
		Connect(GENIDs[1], wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::Base_mode_toggle));
		
		//connect buttons to event handler
		Connect(GENIDs[2], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::OnStopAll));
		Connect(GENIDs[3], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::OnInitAll));
		Connect(GENIDs[4], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::OnRecoverAll));


		vsizer->Add(statuslabel_, 0, wxEXPAND);
		vsizer->Add(btnStopAll, 0, wxEXPAND);
		vsizer->Add(btnInitAll, 0, wxEXPAND);
		vsizer->Add(btnRecoverAll, 0, wxEXPAND);
		
		vsizer->Add(cbPlanning_, 0, wxEXPAND);
		vsizer->Add(cbBaseDiff_, 0, wxEXPAND);	
		
		return vsizer;
	}

	//creates the 'SMACH' box with predefined widgets
	wxSizer* RvizMovementButtonsPanel::CreatesbSMACH()
	{
		//Widget IDs
		const int SMACHIDs[2] = {101, 102};		
		
		wxSizer *vsizer         = new wxStaticBoxSizer(wxVERTICAL, this, wxT("SMACH"));

		wxStaticText *status    = new wxStaticText(this, wxID_ANY, wxT("\n no file selected"));
		wxButton *loadsmach     = new wxButton(this, SMACHIDs[5], wxT("load"), wxDefaultPosition, /*wxDefaultSize*/wxSize(30,29), wxBU_EXACTFIT);
		wxButton *runsmach      = new wxButton(this, SMACHIDs[6], wxT("run"), wxDefaultPosition, /*wxDefaultSize*/wxSize(30,29), wxBU_EXACTFIT);
		//wxFileDialog *openFile  = new wxFileDialog(this);
		
		loadsmach->Enable(true); 
		runsmach->Enable(true);
		/*
		//connect buttons to event handler
		Connect(GENIDs[2], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::OnStopAll));
		Connect(GENIDs[3], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::OnInitAll));
		Connect(GENIDs[4], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::OnRecoverAll));
		*/
		vsizer->Add(status, 0, wxEXPAND);
		vsizer->Add(loadsmach, 0, wxEXPAND);
		vsizer->Add(runsmach, 0, wxEXPAND);
		
		return vsizer;
	}
 
	//creates a 'Button_Description'
	Button_Description RvizMovementButtonsPanel::Descripe_button(const int &ID, XmlRpc::XmlRpcValue data, const std::string &component_name) const
    {
		Button_Description btn;
		
		btn.id = ID;
		btn.button_name   = TostlString(data[0]);
		btn.function_name = TostlString(data[1]);
		btn.args.first    = component_name;
		btn.args.second   = TostlString(data[2]);
		
		return btn;
	}
	
	//adds a visual wxButton and connects it to a event-handling function
	wxButton* RvizMovementButtonsPanel::AddButton(const int &ID, const wxString &name)
	{
		wxButton *but = new wxButton(this, ID, name, wxDefaultPosition, wxSize(30,29)/*wxDefaultSize*/, wxBU_EXACTFIT, wxDefaultValidator, name);
		
		but->Enable(true);
		
		Connect(ID, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(RvizMovementButtonsPanel::OnClick));
		 
		return but;
	}

	//event handler for buttons from the config file on ROS parameter server
	void RvizMovementButtonsPanel::OnClick(wxCommandEvent& event)
	{			    	
		Button_Description pressed_button;
		std::string group_name;
		
		pressed_button.id = event.GetId();
		
		//use the event id to get more information about the pressed button		
		for(Groupdict::const_iterator group_ci = groups_.begin(); group_ci != groups_.end(); group_ci++)
		{
			for(Buttonlist::const_iterator button_ci = group_ci->second.begin(); button_ci != group_ci->second.end(); button_ci++)
			{
				if((button_ci->id == pressed_button.id))
				{
					pressed_button.button_name = (button_ci->button_name);
					pressed_button.function_name = (button_ci->function_name);
					pressed_button.args = (button_ci->args);
					group_name = group_ci->first;
				}
			}
		}
		
		//std::cout << "\n\npressed button: \n"<<"id: "<< pressed_button.id <<"\nbutton_name: "<< pressed_button.button_name <<"\nfunction_name: "<< pressed_button.function_name << "\nargs: " << pressed_button.args.first <<", "<<pressed_button.args.second << "\n\n";
		
		ROS_INFO("'%s-%s' clicked", group_name.c_str(), pressed_button.button_name.c_str());
		
		cob_script_server::ScriptGoal goal;
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
		
		//Fill in goal here
		action_client_->sendGoal(goal);
		action_client_->waitForResult(ros::Duration(1.0));
		if(action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Action succeeded!");
		}
		
		ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());
	}
	
	//event handler for the general 'stop all' Button
	void RvizMovementButtonsPanel::OnStopAll(wxCommandEvent &event)
	{
		cob_script_server::ScriptGoal goal;
		bool success = true; 
		
		goal.function_name  = "stop";
		
		for(Stringlist::const_iterator ci = stop_buttons_.begin(); ci != stop_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
			
		    action_client_->sendGoal(goal);
		    action_client_->waitForResult(ros::Duration(1.0));
		    
		    if("SUCCEEDED" != action_client_->getState().toString())
		    {
				success = false;
				ROS_INFO("Warning: Component %s not properly stopped", goal.component_name.c_str());
			}
		}
		
		if(false != success)
		{ 
			ROS_INFO("All components stopped");
		}
		else
		{
			ROS_INFO("Error during stop!");
		}
	}
	
	//event handler for the general 'init all' Button
	void RvizMovementButtonsPanel::OnInitAll(wxCommandEvent &event)
	{
		cob_script_server::ScriptGoal goal;
		bool success = true; 
		
		goal.function_name = "init";
		
		for(Stringlist::const_iterator ci = init_buttons_.begin(); ci != init_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
		    
		    action_client_->sendGoal(goal);
		    action_client_->waitForResult(ros::Duration(1.0));
		    
		    if("SUCCEEDED" != action_client_->getState().toString())
		    {
				success = false;
				ROS_INFO("Warning: Component %s not properly initialized", goal.component_name.c_str());
			}
		}
		
		if(false != success)
		{ 
			ROS_INFO("All components initialized");
		}
		else
		{
			ROS_INFO("Error during initialization!");
		}
	}
	
	//event handler for the general 'recover all' Button
	void RvizMovementButtonsPanel::OnRecoverAll(wxCommandEvent &event)
	{
		cob_script_server::ScriptGoal goal;
		bool success = true; 
		
		goal.function_name = "recover";
		
		for(Stringlist::const_iterator ci = recover_buttons_.begin(); ci != recover_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
		    
		    action_client_->sendGoal(goal);
		    action_client_->waitForResult(ros::Duration(1.0));
		    
		    if("SUCCEEDED" != action_client_->getState().toString())
		    {
				success = false;
				ROS_INFO("Warning: Component %s not properly recovered", goal.component_name.c_str());
			}
		}
		
		if(false != success)
		{ 
			ROS_INFO("All components recovered");
		}
		else
		{
			ROS_INFO("Error during recovery!");
		}
	}
	
	//event handler for the 'Planning' checkbox
	inline void RvizMovementButtonsPanel::Planned_toggle(wxCommandEvent &event)
	{
		planning_enabled = !planning_enabled;
	}
	
	//event handler for the 'Base Diff' checkbox
	inline void RvizMovementButtonsPanel::Base_mode_toggle(wxCommandEvent &event)
	{
		base_diff_enabled = !base_diff_enabled;
	}

	//converts a std::string into a wxString
	inline wxString RvizMovementButtonsPanel::TowxString(const std::string &temp) const
	{
		return wxString(temp.c_str(), wxConvUTF8);
	}
	
	/*coverts a XmlRpcValue of 'TypeString' or a wxString into std::string
	  
	  note:
	  converting a XmlRpcValue of 'TypeString' to a std::string won't work within a 'operator =' statement.
	
	  however, handing over the XmlRpcValue as a std::string parameter works perfectly 
	*/	  
	inline std::string RvizMovementButtonsPanel::TostlString(const std::string &temp, const wxString &wxtemp) const
	{	
		if(false == wxtemp.IsEmpty())
		{
			return (std::string(wxtemp.mb_str()));
		}
		else
		{
			return temp;
		}
		
		return temp;
	}

}

