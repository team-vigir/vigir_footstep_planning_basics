#include <vigir_footstep_planning_plugins/step_plan_msg_plugin.h>



namespace vigir_footstep_planning
{
StepPlanMsgPlugin::StepPlanMsgPlugin(const std::string& name)
  : Plugin(name)
{
}

StepPlanMsgPlugin::~StepPlanMsgPlugin()
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::StepPlanMsgPlugin, vigir_footstep_planning::StepPlanMsgPlugin)

