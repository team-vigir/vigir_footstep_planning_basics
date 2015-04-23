#include <vigir_footstep_planning_lib/plugins/step_plan_msg_plugin.h>

namespace vigir_footstep_planning
{
StepPlanMsgPlugin::StepPlanMsgPlugin(const std::string& name)
  : Plugin(name, "step_plan_msg_plugin")
{
}

StepPlanMsgPlugin::StepPlanMsgPlugin()
  : Plugin("default_step_plan_msg_plugin", "step_plan_msg_plugin")
{
}

StepPlanMsgPlugin::~StepPlanMsgPlugin()
{
}
}
