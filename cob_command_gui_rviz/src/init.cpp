#include "rviz/plugin/type_registry.h"

#include "cob_rviz_movement_control/rviz_movement_buttons.h"

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<rviz::RvizMovementButtons>("RvizMovementButtons");
}
