#ifndef RVIZ_MOVEMENT_BUTTONS_H
#define RVIZ_MOVEMENT_BUTTONS_H

#include <wx/wx.h>
#include <wx/dialog.h>

#include <ros/ros.h>
#include <rviz/display.h>

#include <string.h>

#include "cob_rviz_movement_control/rviz_movement_buttons_panel.h"

namespace rviz
{
	class RvizMovementButtons : public Display
	{
	
	public:
	    //constructor 
	    RvizMovementButtons(const std::string& name, VisualizationManager* manager);
	    ~RvizMovementButtons();
	    
	    void onEnable();
	    void onDisable();
	    
	    
	    //nötig?
	    void targetFrameChanged()
	    {
	    }
	    //nötig?
	    void fixedFrameChanged()
	    {
	    }
		
	protected:
		
	    //ros::ServiceServer service_start_;
	    //ros::ServiceServer service_timeout_;
		
	    RvizMovementButtonsPanel *panel_;
		
	    wxFrame *frame_;
		
	};//RvizMovementButtons
	
}

#endif //RVIZ_MOVEMENT_BUTTONS_H
