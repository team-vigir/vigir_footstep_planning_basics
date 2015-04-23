#include <vigir_footstep_planning_lib/plugins/reachability_plugin.h>

namespace vigir_footstep_planning
{
ReachabilityPlugin::ReachabilityPlugin(const std::string& name, const ParameterSet& params)
  : Plugin(name, "reachability_plugin", params)
{
}

ReachabilityPlugin::ReachabilityPlugin(const std::string& name)
  : Plugin(name, "reachability_plugin")
{
}

ReachabilityPlugin::~ReachabilityPlugin()
{
}

bool ReachabilityPlugin::isUnique() const
{
  return false;
}
}
