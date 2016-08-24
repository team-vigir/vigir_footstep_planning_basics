#include <vigir_footstep_planning_plugins/plugin_aggregators/step_cost_estimator.h>



namespace vigir_footstep_planning
{
StepCostEstimator::StepCostEstimator()
  : ExtendedPluginAggregator<StepCostEstimator, StepCostEstimatorPlugin>("StepCostEstimator")
{
}

bool StepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& risk) const
{
  cost = 0.0;
  double cost_multiplier = 1.0;
  risk = 0.0;
  double risk_multiplier = 1.0;

  //ROS_INFO("-----------------------------------");
  for (StepCostEstimatorPlugin::Ptr plugin : getPlugins())
  {
    if (!plugin)
      continue;

    double c, c_m, r, r_m;
    if (plugin->getCost(left_foot, right_foot, swing_foot, c, c_m, r, r_m))
    {
      cost += c;
      cost_multiplier *= c_m;
      risk += r;
      risk_multiplier *= r_m;
      //ROS_INFO("[%s]: %.3f %.3f %.3f %.3f", step_cost_estimator->getName().c_str(), c, r, c_m, r_m);
    }
    else
      return false;
  }

  cost *= cost_multiplier;
  risk *= risk_multiplier;

  return true;
}

bool StepCostEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, float& cost, float& risk) const
{
  double cost_d, risk_d;
  bool result = getCost(left_foot, right_foot, swing_foot, cost_d, risk_d);
  cost = static_cast<float>(cost_d);
  risk = static_cast<float>(risk_d);
  return result;
}
}
