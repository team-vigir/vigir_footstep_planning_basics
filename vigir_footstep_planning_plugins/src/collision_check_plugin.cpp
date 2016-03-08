#include <vigir_footstep_planning_plugins/collision_check_plugin.h>

namespace vigir_footstep_planning
{
CollisionCheckPlugin::CollisionCheckPlugin(const std::string& name)
  : Plugin(name)
  , collision_check_flag_(0)
{
}

bool CollisionCheckPlugin::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!Plugin::initialize(nh, params))
    return false;

  getPluginParam("collision_check_flag", (int&)collision_check_flag_, -1, true);

  return true;
}

bool CollisionCheckPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!Plugin::loadParams(params))
    return false;

  // check if
  unsigned int collision_check_mask;
  params.getParam("collision_check/collision_check_mask", (int&)collision_check_mask);
  collision_check_enabled_ = this->collision_check_flag_ & collision_check_mask;

  return true;
}

void CollisionCheckPlugin::reset()
{
}

bool CollisionCheckPlugin::isUnique() const
{
  return false;
}

bool CollisionCheckPlugin::isCollisionCheckAvailable() const
{
  return collision_check_enabled_;
}
}
