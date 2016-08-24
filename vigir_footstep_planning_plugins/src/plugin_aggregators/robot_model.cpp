#include <vigir_footstep_planning_plugins/plugin_aggregators/robot_model.h>



namespace vigir_footstep_planning
{
RobotModel::RobotModel()
  : ExtendedPluginAggregator<RobotModel, ReachabilityPlugin>("RobotModel")
{
}

bool RobotModel::isReachable(const State& current, const State& next) const
{
  for (ReachabilityPlugin::Ptr plugin : getPlugins())
  {
    if (plugin && !plugin->isReachable(current, next))
      return false;
  }
  return true;
}

bool RobotModel::isReachable(const State& left, const State& right, const State& swing) const
{
  for (ReachabilityPlugin::Ptr plugin : getPlugins())
  {
    if (plugin && !plugin->isReachable(left, right, swing))
      return false;
  }
  return true;
}
}
