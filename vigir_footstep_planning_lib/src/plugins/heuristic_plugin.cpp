#include <vigir_footstep_planning_lib/plugins/heuristic_plugin.h>

namespace vigir_footstep_planning
{
HeuristicPlugin::HeuristicPlugin(const std::string& name, const ParameterSet& params)
  : Plugin(name, "heuristic_plugin", params)
{
}

HeuristicPlugin::HeuristicPlugin(const std::string& name)
  : Plugin(name, "heuristic_plugin")
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
}
