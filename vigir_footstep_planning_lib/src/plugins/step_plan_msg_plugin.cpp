#include <vigir_footstep_planning_lib/plugins/step_plan_msg_plugin.h>

namespace vigir_footstep_planning
{
StepPlanMsgPlugin::StepPlanMsgPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name, "step_plan_msg_plugin")
{
}

StepPlanMsgPlugin::StepPlanMsgPlugin()
  : vigir_pluginlib::Plugin("default_step_plan_msg_plugin", "step_plan_msg_plugin")
{
}

StepPlanMsgPlugin::~StepPlanMsgPlugin()
{
}
}
