#include <vigir_footstep_planning_plugins/step_plan_msg_plugin.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
StepPlanMsgPlugin::StepPlanMsgPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name, "vigir_footstep_planning::StepPlanMsgPlugin")
{
}

StepPlanMsgPlugin::~StepPlanMsgPlugin()
{
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::StepPlanMsgPlugin, vigir_footstep_planning::StepPlanMsgPlugin)

