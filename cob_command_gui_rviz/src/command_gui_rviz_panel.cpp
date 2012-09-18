#include "cob_command_gui_rviz/command_gui_rviz_panel.h"

#include <wx/event.h>
#include <XmlRpcValue.h>

namespace rviz
{	
	
	//constructor
	CommandGuiRvizPanel::CommandGuiRvizPanel(wxWindow *parent, const wxString& title):
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
			action_client_ = new actionlib::SimpleActionClient<cob_script_server::ScriptAction>("cob_command_gui_rviz", true);		
		}
		else
		{
			//no parameter available -> Error message
			statuslabel_ = new wxStaticText(this, wxID_ANY, wxT("\n\tStatus: Error - Parameter does not exist on ROS Parameter Server! \n\n"));	
		}	
	}

	//destructor
	CommandGuiRvizPanel::~CommandGuiRvizPanel() 
	{
	}
	
	//try to load button parameters from ROS Parameter Server
	bool CommandGuiRvizPanel::Getparams()
	{	
		std::string param_prefix = "/control_buttons";		
		
		if(false == (ros::param::has(param_prefix)))
		{
			ROS_DEBUG_ONCE("Parameter '%s' does not exist on ROS Parameter Server, aborting...\n", param_prefix.c_str());			
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
		    static int id = 101;    		        		    
		 	 
			Buttonlist current_buttons;	
					 
			// create buttons in group		 
		    for(int j=0; j < buttons.size(); j++)
		    {
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
					ROS_DEBUG_ONCE("parameter %s does not exist on ROS Parameter Server, no nav buttons will be available!\n", param_prefix.c_str());		
				}
			}
			
			groups_.push_back(std::pair<std::string, Buttonlist>(TostlString(group_name),current_buttons));
		}				
		
		//uniquify lists
		stop_buttons_.unique();
		init_buttons_.unique();
		recover_buttons_.unique();
		
		return true;
	}	
	
	//creates the graphical user interface
	void CommandGuiRvizPanel::Creategui()
	{	
		//main sizer
		wxSizer *mainsizer = new wxBoxSizer(wxHORIZONTAL);
		
		//create general box
		sizers_.push_back(CreatesbGeneral());
		
		//create boxes and buttons inside them
		for(Grouplist::const_iterator group_ci = groups_.begin(); group_ci != groups_.end(); group_ci++)
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
	wxSizer* CommandGuiRvizPanel::CreatesbGeneral()
	{			
		//Widget IDs
		const int GENIDs[5] = {001, 002, 003, 004, 005};		
		
		wxSizer *vsizer = new wxStaticBoxSizer(wxVERTICAL, this, wxT("general"));
		
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
		Connect(GENIDs[0], wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(CommandGuiRvizPanel::Planned_toggle));
		Connect(GENIDs[1], wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(CommandGuiRvizPanel::Base_mode_toggle));
		
		//connect buttons to event handler
		Connect(GENIDs[2], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(CommandGuiRvizPanel::OnStopAll));
		Connect(GENIDs[3], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(CommandGuiRvizPanel::OnInitAll));
		Connect(GENIDs[4], wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(CommandGuiRvizPanel::OnRecoverAll));


		vsizer->Add(statuslabel_, 0, wxEXPAND);
		vsizer->Add(btnStopAll, 0, wxEXPAND);
		vsizer->Add(btnInitAll, 0, wxEXPAND);
		vsizer->Add(btnRecoverAll, 0, wxEXPAND);
		
		vsizer->Add(cbPlanning_, 0, wxEXPAND);
		vsizer->Add(cbBaseDiff_, 0, wxEXPAND);	
		
		return vsizer;
	}
 
	//creates a 'Button_Description'
	Button_Description CommandGuiRvizPanel::Descripe_button(const int &ID, XmlRpc::XmlRpcValue data, const std::string &component_name) const
    {
		Button_Description btn;
		
		btn.id = ID;
		btn.button_name   = TostlString(data[0]);
		btn.function_name = TostlString(data[1]);
		btn.args.first    = component_name;
		btn.args.second   = TostlString(data[2]);
		
		return btn;
	}
	
	//creates a wxButton and connects it to a event-handler
	wxButton* CommandGuiRvizPanel::AddButton(const int &ID, const wxString &name)
	{
		wxButton *but = new wxButton(this, ID, name, wxDefaultPosition, wxSize(30,29)/*wxDefaultSize*/, wxBU_EXACTFIT, wxDefaultValidator, name);
		
		but->Enable(true);
		
		Connect(ID, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(CommandGuiRvizPanel::OnClick));
		 
		return but;
	}

	//event handler for buttons from the config file on ROS parameter server
	void CommandGuiRvizPanel::OnClick(wxCommandEvent& event)
	{			    	
		Button_Description pressed_button;
		std::string group_name;
		
		pressed_button.id = event.GetId();
		
		//use the button_id to get more information about the pressed button		
		for(Grouplist::const_iterator group_ci = groups_.begin(); group_ci != groups_.end(); group_ci++)
		{
			for(Buttonlist::const_iterator button_ci = group_ci->second.begin(); button_ci != group_ci->second.end(); button_ci++)
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
			//set status to 'ok'
			statuslabel_->SetLabel(wxT("\n Status: OK"));
			
			this->GetSizer()->Layout();
		}
		else
		{
			//set status to 'no connection'
			statuslabel_->SetLabel(wxT("\nStatus: \nno connect. to action server!\n\n"));
			
			this->GetSizer()->Layout();
			
			ROS_DEBUG_ONCE("Error! - no connection to action server!\n");
		}
		
		ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());
	}
	
	//event handler for the 'stop all' Button
	void CommandGuiRvizPanel::OnStopAll(wxCommandEvent &event)
	{
		cob_script_server::ScriptGoal goal;
		bool success = true; 
		
		goal.function_name  = "stop";
		
		ROS_INFO("'stop all' clicked");
		
		for(Stringlist::const_iterator ci = stop_buttons_.begin(); ci != stop_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
			
		    action_client_->sendGoal(goal);
		    action_client_->waitForResult(ros::Duration(1.0));
		    
		    if("SUCCEEDED" != action_client_->getState().toString())
		    {
				success = false;
				ROS_INFO("Warning: Component %s not properly stopped\n", goal.component_name.c_str());
			}
			
			ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());
		}
		
		if(false != success)
		{ 
			ROS_INFO("All components stopped\n");
			
			statuslabel_->SetLabel(wxT("\n Status: OK"));
			this->GetSizer()->Layout();
		}
		else
		{
			//set status to 'no connection'
			statuslabel_->SetLabel(wxT("\nStatus: \nno connect. to action server!\n\n"));
			
			this->GetSizer()->Layout();
			ROS_DEBUG_ONCE("Error! - no connection to action server!\n");
		}
	}
	
	//event handler for the 'init all' Button
	void CommandGuiRvizPanel::OnInitAll(wxCommandEvent &event)
	{
		cob_script_server::ScriptGoal goal;
		bool success = true; 
		
		goal.function_name = "init";
		
		ROS_INFO("'init all' clicked");
		
		for(Stringlist::const_iterator ci = init_buttons_.begin(); ci != init_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
		    
		    action_client_->sendGoal(goal);
		    action_client_->waitForResult(ros::Duration(1.0));
		    
		    if("SUCCEEDED" != action_client_->getState().toString())
		    {
				success = false;
				ROS_INFO("Warning: Component %s not properly initialized\n", goal.component_name.c_str());
			}
			
			ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());
		}
		
		if(false != success)
		{ 
			statuslabel_->SetLabel(wxT("\n Status: OK"));
			this->GetSizer()->Layout();
			
			ROS_INFO("All components initialized\n");
		}
		else
		{
			//set status to 'no connection'
			statuslabel_->SetLabel(wxT("\nStatus: \nno connect. to action server!\n\n"));
			this->GetSizer()->Layout();
			
			ROS_DEBUG_ONCE("Error! - no connection to action server!\n");
		}
	}
	
	//event handler for the 'recover all' Button
	void CommandGuiRvizPanel::OnRecoverAll(wxCommandEvent &event)
	{
		cob_script_server::ScriptGoal goal;
		bool success = true; 
		
		goal.function_name = "recover";
		
		ROS_INFO("'recover all' clicked");
		
		for(Stringlist::const_iterator ci = recover_buttons_.begin(); ci != recover_buttons_.end(); ci++)
		{
			goal.component_name = *ci;
		    
		    action_client_->sendGoal(goal);
		    action_client_->waitForResult(ros::Duration(1.0));
		    
		    if("SUCCEEDED" != action_client_->getState().toString())
		    {
				success = false;
				ROS_INFO("Warning: Component %s not properly recovered\n", goal.component_name.c_str());
			}
			
			ROS_INFO("Current State: %s\n", action_client_->getState().toString().c_str());
		}
		
		if(false != success)
		{ 
			statuslabel_->SetLabel(wxT("\n Status: OK"));
			this->GetSizer()->Layout();
			
			ROS_INFO("All components recovered\n");
		}
		else
		{
			//set status to 'no connection'
			statuslabel_->SetLabel(wxT("\nStatus: \nno connect. to action server!\n\n"));
			this->GetSizer()->Layout();
			
			ROS_DEBUG_ONCE("Error! - no connection to action server!\n");
		}
	}
	
	//event handler for the 'planning' checkbox
	inline void CommandGuiRvizPanel::Planned_toggle(wxCommandEvent &event)
	{
		planning_enabled = !planning_enabled;
	}
	
	//event handler for the 'base diff' checkbox
	inline void CommandGuiRvizPanel::Base_mode_toggle(wxCommandEvent &event)
	{
		base_diff_enabled = !base_diff_enabled;
	}

	//converts a std::string into a wxString
	inline wxString CommandGuiRvizPanel::TowxString(const std::string &temp) const
	{
		return wxString(temp.c_str(), wxConvUTF8);
	}
	
	//coverts a XmlRpcValue of 'TypeString' or a wxString into std::string
	inline std::string CommandGuiRvizPanel::TostlString(XmlRpc::XmlRpcValue temp, wxString wxtemp) const
	{	
		if(false == wxtemp.IsEmpty())
		{
			return std::string(wxtemp.mb_str());
		}
		else
		{
			return static_cast<std::string>(temp);
		}
	}

}
