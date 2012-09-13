#ifndef RVIZ_MOVEMENT_BUTTONS_PANEL_H
#define RVIZ_MOVEMENT_BUTTONS_PANEL_H

#include <wx/panel.h>
#include <wx/button.h>
#include <wx/stattext.h>
#include <wx/checkbox.h>
#include <wx/sizer.h>

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
	typedef std::map<std::string, Buttonlist> Groupdict; 
	typedef std::vector<wxSizer*> Sizerlist;
	typedef std::list<std::string> Stringlist;



	class RvizMovementButtonsPanel : public wxPanel
	{
	public:
		
		RvizMovementButtonsPanel(wxWindow *parent, const wxString&title);
		~RvizMovementButtonsPanel();
	
	protected:	 	    

//=========================================MEMBERS======================================================	    
		
		//widgets for 'general' box
		wxStaticText *statuslabel_;
		wxCheckBox *cbPlanning_;
		wxCheckBox *cbBaseDiff_;
		
		//containers
		Sizerlist sizers_;
		Groupdict groups_;
		
		Stringlist stop_buttons_;
		Stringlist init_buttons_; 
		Stringlist recover_buttons_;

		//checkbox status
		bool planning_enabled;	
		bool base_diff_enabled;

		//actionclient    		
        actionlib::SimpleActionClient<cob_script_server::ScriptAction>* action_client_;	

//=========================================METHODS======================================================		
		
		bool Getparams();
		void Creategui();
		wxSizer *CreatesbGeneral();
		wxSizer *CreatesbSMACH();
		
		//Stringlist uniquify_list(Stringlist to_uniquify,idfun=0);
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
		inline std::string TostlString(const std::string &temp = "", const wxString &wxtemp =wxT("")) const;

	};
}
#endif 

/*

{
  "group1": {
    "buttons": [
      [
        "stop", 
        "stop", 
        "omni"
      ], 
      [
        "init", 
        "trigger", 
        "init"
      ], 
      [
        "recover", 
        "trigger", 
        "recover"
      ]
    ], 
    "group_name": "base", 
    "component_name": "base"
  }
}
* 
XMLRPCStruktur:
* 
* Top: TypeStruct(groupkey,xmlrpcvalue)
* 
* 	Botton: TypeStruct(key,xmlrpcvalue)
* 		1: TypeArray(von Arrays buttons
* 			Down: TypeArray(string)
* 		2: TypeString group_name
* 		3: TypeString comp_name

		if(group_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) 
		{
			ROS_INFO("Typestruct");
		}
*/ 

/*
buttons.append(self.CreateButton(button[0],self.sss.move,component_name,button[2]))

	
	  def CreateButton(self,button_name,function,component_name,parameter_name):
		button = (button_name,function,(component_name,parameter_name,False))
		return button
		 
	 for aname, func, args in actions:
        panel.addButton(text=aname, command=lambda f=func, a=args: start(f, a))
        * 
	def start(func, args):
	  global planning_enabled
	  global base_diff_enabled
	  largs = list(args)
	  if(largs[0] == "arm"):
		if(planning_enabled):
		  largs.append("planned")
	  if(largs[0] == "base"):
		if(base_diff_enabled):
		largs.append("diff")	
	  #print "Args", tuple(largs)
	  thread.start_new_thread(func,tuple(largs))
	
		 
	  def addButton(self, text, command):
		but = gtk.Button(text)  
		but.connect("clicked", startGTK, command) //command ist optional, clicked ist name & startGtk ist handler
		#but.set_size_request(120,-1)
		self.vbox.pack_start(but, False, False, 5)	
	
    Buttonsdescription:
 * 
 *  1. button name
 *  2. function
 *  3. liste aus (component_name, parameter_name, false)
 *  
 */ 
		
