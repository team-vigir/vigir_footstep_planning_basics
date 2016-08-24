#include <vigir_footstep_planning_plugins/plugin_aggregators/heuristic.h>



namespace vigir_footstep_planning
{
Heuristic::Heuristic()
  : ExtendedPluginAggregator<Heuristic, HeuristicPlugin>("Heuristic")
{
}

double Heuristic::getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const
{
  double h = 0.0;

  for (HeuristicPlugin::Ptr plugin : getPlugins())
  {
    if (plugin)
      h += plugin->getHeuristicValue(from, to, start, goal);
  }

  return h;
}
}
