#include <vigir_footstep_planning_plugins/heuristic_plugin.h>



namespace vigir_footstep_planning
{
HeuristicPlugin::HeuristicPlugin(const std::string& name)
  : Plugin(name)
{
}

HeuristicPlugin::~HeuristicPlugin()
{
}

void HeuristicPlugin::reset()
{
}

bool HeuristicPlugin::isUnique() const
{
  return false;
}

bool HeuristicPlugin::loadParams(const vigir_generic_params::ParameterSet& global_params)
{
  if (!Plugin::loadParams(global_params))
    return false;

  global_params.getParam("max_heuristic_value", max_heuristic_value_);
  return true;
}
}
