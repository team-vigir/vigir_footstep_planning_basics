#include <vigir_footstep_planning_plugins/step_cost_estimator_plugin.h>

namespace vigir_footstep_planning
{
StepCostEstimatorPlugin::StepCostEstimatorPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name, "vigir_footstep_planning::StepCostEstimatorPlugin")
{
}

StepCostEstimatorPlugin::~StepCostEstimatorPlugin()
{
}

bool StepCostEstimatorPlugin::isUnique() const
{
  return false;
}
}
