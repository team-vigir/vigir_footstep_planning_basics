#include <vigir_footstep_planning_lib/plugins/collision_check_plugin.h>

namespace vigir_footstep_planning
{
CollisionCheckPlugin::CollisionCheckPlugin(const std::string& name, const std::string& type, const vigir_generic_params::ParameterSet& params, unsigned int collision_check_flag)
  : vigir_pluginlib::Plugin(name, type, params)
  , collision_check_flag(collision_check_flag)
{
}

CollisionCheckPlugin::CollisionCheckPlugin(const std::string& name, const std::string& type, unsigned int collision_check_flag)
  : vigir_pluginlib::Plugin(name, type)
  , collision_check_flag(collision_check_flag)
{
}

void CollisionCheckPlugin::reset()
{
}

void CollisionCheckPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  Plugin::loadParams(params);

  unsigned int collision_check_flag;
  params.getParam("collision_check/collision_check_flag", (int&)collision_check_flag);
  collision_check_enabled = collision_check_flag & this->collision_check_flag;
}

bool CollisionCheckPlugin::isUnique() const
{
  return false;
}

bool CollisionCheckPlugin::isCollisionCheckAvailable() const
{
  return collision_check_enabled;
}
}
