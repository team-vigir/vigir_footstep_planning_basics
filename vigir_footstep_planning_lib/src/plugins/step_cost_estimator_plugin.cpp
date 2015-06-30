#include <vigir_footstep_planning_lib/plugins/step_cost_estimator_plugin.h>

namespace vigir_footstep_planning
{
StepCostEstimatorPlugin::StepCostEstimatorPlugin(const std::string& name, const vigir_generic_params::ParameterSet& params)
  : vigir_pluginlib::Plugin(name, "step_cost_estimator_plugin", params)
{
}

StepCostEstimatorPlugin::StepCostEstimatorPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name, "step_cost_estimator_plugin")
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
