#include <vigir_footstep_planning_plugins/plugin_aggregators/state_generator.h>



namespace vigir_footstep_planning
{
StateGenerator::StateGenerator()
  : ExtendedPluginAggregator<StateGenerator, StateGeneratorPlugin>("StateGenerator")
{
}

std::list<PlanningState::Ptr> StateGenerator::generatePredecessor(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;
  return result;
}

std::list<PlanningState::Ptr> StateGenerator::generateSuccessor(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;
  return result;
}
}
