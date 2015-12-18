#include <vigir_footstep_planning_basic_plugins/reachability_plugin.h>

namespace vigir_footstep_planning
{
ReachabilityPlugin::ReachabilityPlugin(const std::string& name)
  : Plugin(name, "vigir_footstep_planning::ReachabilityPlugin")
{
}

ReachabilityPlugin::~ReachabilityPlugin()
{
}

bool ReachabilityPlugin::isUnique() const
{
  return false;
}

bool ReachabilityPlugin::isReachable(const State& left_foot, const State& right_foot, const State& swing_foot) const
{
  if (swing_foot.getLeg() == LEFT)
    return isReachable(right_foot, swing_foot);
  else
    return isReachable(left_foot, swing_foot);
}
}
