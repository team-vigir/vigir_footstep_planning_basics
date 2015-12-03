#include <vigir_footstep_planning_lib/plugins/heuristic_plugin.h>

namespace vigir_footstep_planning
{
HeuristicPlugin::HeuristicPlugin(const std::string& name, const vigir_generic_params::ParameterSet& params)
  : vigir_pluginlib::Plugin(name, "vigir_footstep_planning::HeuristicPlugin", params)
{
}

HeuristicPlugin::HeuristicPlugin(const std::string& name)
  : vigir_pluginlib::Plugin(name, "vigir_footstep_planning::HeuristicPlugin")
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

void HeuristicPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  params.getParam("max_heuristic_value", max_heuristic_value);
}
}
