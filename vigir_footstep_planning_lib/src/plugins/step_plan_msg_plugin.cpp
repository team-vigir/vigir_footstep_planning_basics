#include <vigir_footstep_planning_lib/plugins/step_plan_msg_plugin.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
StepPlanMsgPlugin::StepPlanMsgPlugin(const std::string& name, const vigir_generic_params::ParameterSet& params)
  : vigir_pluginlib::Plugin(name, "vigir_footstep_planning::StepPlanMsgPlugin", params)
{
}

StepPlanMsgPlugin::StepPlanMsgPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name, "vigir_footstep_planning::StepPlanMsgPlugin")
{
}

StepPlanMsgPlugin::StepPlanMsgPlugin()
  : StepPlanMsgPlugin("default_step_plan_msg_plugin")
{
}

StepPlanMsgPlugin::~StepPlanMsgPlugin()
{
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::StepPlanMsgPlugin, vigir_footstep_planning::StepPlanMsgPlugin)

