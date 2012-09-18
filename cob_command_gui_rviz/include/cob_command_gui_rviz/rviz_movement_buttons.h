#ifndef RVIZ_MOVEMENT_BUTTONS_H
#define RVIZ_MOVEMENT_BUTTONS_H

#include <wx/wx.h>
#include <wx/dialog.h>

#include <ros/ros.h>
#include <rviz/display.h>

#include <string.h>

#include "cob_command_gui_rviz/rviz_movement_buttons_panel.h"

namespace rviz
{
	class RvizMovementButtons : public Display
	{
	public:
	
	    RvizMovementButtons(const std::string& name, VisualizationManager* manager);
	    ~RvizMovementButtons();
	    
	    void onEnable();
	    void onDisable();
	    
	    void targetFrameChanged()
	    {
	    }
	    
	    void fixedFrameChanged()
	    {
	    }
	   
	
	protected:
		
	    RvizMovementButtonsPanel *panel_;
	    wxFrame *frame_;
	};
}

#endif 

