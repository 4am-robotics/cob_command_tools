#include "rviz/plugin/type_registry.h"

#include "cob_command_gui_rviz/command_gui_rviz.h"

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
	reg->registerDisplay<rviz::CommandGuiRviz>("CommandGuiRviz");
}
