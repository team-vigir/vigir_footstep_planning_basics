#include <vigir_footstep_planning_plugins/plugin_aggregators/state_generator.h>



namespace vigir_footstep_planning
{
StateGenerator::StateGenerator()
  : ExtendedPluginAggregator<StateGenerator, StateGeneratorPlugin>("StateGenerator")
{
}

std::list<PlanningState::Ptr> StateGenerator::generatePredecessors(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;

  for (StateGeneratorPlugin::Ptr plugin : getPlugins())
  {
    if (plugin)
      result.splice(result.end(), plugin->generatePredecessors(state));
  }

  return result;
}

std::list<PlanningState::Ptr> StateGenerator::generateSuccessors(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;

  for (StateGeneratorPlugin::Ptr plugin : getPlugins())
  {
    if (plugin)
      result.splice(result.end(), plugin->generateSuccessors(state));
  }

  return result;
}
}
