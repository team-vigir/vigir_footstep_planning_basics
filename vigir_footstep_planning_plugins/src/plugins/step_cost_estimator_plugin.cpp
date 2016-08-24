#include <vigir_footstep_planning_plugins/plugins/step_cost_estimator_plugin.h>



namespace vigir_footstep_planning
{
StepCostEstimatorPlugin::StepCostEstimatorPlugin(const std::string& name)
  : Plugin(name)
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
