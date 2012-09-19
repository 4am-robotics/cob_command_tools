#ifndef COMMAND_GUI_RVIZ_H
#define COMMAND_GUI_RVIZ_H

#include <wx/wx.h>
#include <wx/dialog.h>

#include <ros/ros.h>
#include <rviz/display.h>

#include <string.h>

#include "cob_command_gui_rviz/command_gui_rviz_panel.h"

namespace rviz
{
	class CommandGuiRviz : public Display
	{
	public:
	
	    CommandGuiRviz(const std::string& name, VisualizationManager* manager);
	    ~CommandGuiRviz();
	    
	    void onEnable();
	    void onDisable();
	    
	    void targetFrameChanged()
	    {
	    }
	    
	    void fixedFrameChanged()
	    {
	    }
	   
	
	protected:
		
	    CommandGuiRvizPanel *panel_;
	    wxFrame *frame_;
	};
}

#endif 

