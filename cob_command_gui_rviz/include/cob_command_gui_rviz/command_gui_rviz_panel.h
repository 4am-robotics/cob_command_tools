#ifndef COMMAND_GUI_RVIZ_PANEL_H
#define COMMAND_GUI_RVIZ_PANEL_H

#include <wx/panel.h>
#include <wx/button.h>
#include <wx/stattext.h>
#include <wx/checkbox.h>
#include <wx/sizer.h>

#include <list>
#include <pair>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <cob_script_server/ScriptAction.h>
#include <actionlib/client/simple_action_client.h>

namespace rviz
{
	struct Button_Description
	{	
		int id;
		std::string button_name;
		std::string function_name;
		std::pair<std::string, std::string> args;
	};

	typedef std::list<Button_Description> Buttonlist;
	typedef std::list< std::pair<std::string, Buttonlist> > Grouplist;
	typedef std::vector<wxSizer*> Sizerlist;
	typedef std::list<std::string> Stringlist;

	class CommandGuiRvizPanel : public wxPanel
	{
	public:
		
		CommandGuiRvizPanel(wxWindow *parent, const wxString&title);
		~CommandGuiRvizPanel();
	
	protected:	 	    
		
		//widgets for 'general' box
		wxStaticText *statuslabel_;
		wxCheckBox *cbPlanning_;
		wxCheckBox *cbBaseDiff_;
		
		//containers
		Sizerlist sizers_;
		Grouplist groups_;
		
		Stringlist stop_buttons_;
		Stringlist init_buttons_; 
		Stringlist recover_buttons_;

		//checkbox status
		bool planning_enabled;	
		bool base_diff_enabled;

		//actionclient    		
        actionlib::SimpleActionClient<cob_script_server::ScriptAction>* action_client_;	
		
		bool Getparams();
		void Creategui();
		wxSizer *CreatesbGeneral();
		
		Button_Description Descripe_button(const int &ID, XmlRpc::XmlRpcValue data, const std::string &component_name) const;
		wxButton* AddButton(const int &ID, const wxString &title);
		
		//event-functions
		void OnClick(wxCommandEvent &event);
		void OnStopAll(wxCommandEvent &event);
		void OnInitAll(wxCommandEvent &event);
		void OnRecoverAll(wxCommandEvent &event);
		inline void Planned_toggle(wxCommandEvent &event);
		inline void Base_mode_toggle(wxCommandEvent &event);

	private:
		//some typecast-functions
		inline wxString TowxString(const std::string &temp) const;
		inline std::string TostlString(XmlRpc::XmlRpcValue temp = XmlRpc::XmlRpcValue(), wxString wxtemp = wxString()) const;
	};
}

#endif 
