#include "cob_rviz_movement_control/rviz_movement_buttons.h"

#include "rviz/visualization_manager.h"     
#include "rviz/window_manager_interface.h" 

namespace rviz 
{
	
	//constructor
	RvizMovementButtons::RvizMovementButtons(const std::string &name, VisualizationManager* manager):
	Display(name, manager),
	frame_(0)
	{
		// Create controls
		wxWindow* parent = 0;
		
		WindowManagerInterface* wm = vis_manager_->getWindowManager();
		
		if(wm)
		{
			parent = wm->getParentWindow();
		}
		else
		{
			frame_ = new wxFrame(0, wxID_ANY, wxString::FromAscii(name.c_str()), wxDefaultPosition, wxDefaultSize, wxMINIMIZE_BOX | wxMAXIMIZE_BOX | wxRESIZE_BORDER | wxCAPTION | wxCLIP_CHILDREN);
			parent = frame_;		
		}
		
		panel_ = new RvizMovementButtonsPanel(parent, wxString());
		
		if(wm)
		{
			wm->addPane(name, panel_);
		}
	}
	
	RvizMovementButtons::~RvizMovementButtons()
	{
	}

	void RvizMovementButtons::onEnable()
	{
		if(frame_)
		{
			frame_->Show(true);
		}
		else
		{
			WindowManagerInterface* wm = vis_manager_->getWindowManager();
			wm->showPane(panel_);
		}
	}
	
	void RvizMovementButtons::onDisable()
	{
		if(frame_)
		{
			frame_->Show(false);
		}
		else
		{
			WindowManagerInterface *wm = vis_manager_->getWindowManager();
			wm->closePane(panel_);
		}
	}
	
}
