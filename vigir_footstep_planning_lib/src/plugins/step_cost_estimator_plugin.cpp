#include <vigir_footstep_planning_lib/plugins/step_cost_estimator_plugin.h>

namespace vigir_footstep_planning
{
StepCostEstimatorPlugin::StepCostEstimatorPlugin(const std::string& name, const ParameterSet& params)
  : Plugin(name, "step_cost_estimator_plugin", params)
{
}

StepCostEstimatorPlugin::StepCostEstimatorPlugin(const std::string& name)
  : Plugin(name, "step_cost_estimator_plugin")
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
