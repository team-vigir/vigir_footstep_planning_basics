#include <vigir_footstep_planning_plugins/plugin_aggregators/world_model.h>

namespace vigir_footstep_planning
{
WorldModel::WorldModel()
  : ExtendedPluginAggregator<WorldModel, CollisionCheckPlugin>("WorldModel")
{
}

void WorldModel::loadPlugins(bool print_warning)
{
  ExtendedPluginAggregator<WorldModel, CollisionCheckPlugin>::loadPlugins(print_warning);

  // get terrain model
  vigir_pluginlib::PluginManager::getPlugin(terrain_model_);
  if (terrain_model_)
  {
    ROS_INFO("[WorldModel] Found terrain model:");
    ROS_INFO("    %s (%s)", terrain_model_->getName().c_str(), terrain_model_->getTypeClass().c_str());
  }
}

bool WorldModel::loadParams(const vigir_generic_params::ParameterSet& params)
{
  bool result = ExtendedPluginAggregator<WorldModel, CollisionCheckPlugin>::loadParams(params);

  if (terrain_model_)
    result &= terrain_model_->loadParams(params);

  return result;
}

void WorldModel::resetPlugins()
{
  ExtendedPluginAggregator<WorldModel, CollisionCheckPlugin>::resetPlugins();

  if (terrain_model_)
    terrain_model_->reset();
}

bool WorldModel::isAccessible(const State& s) const
{
  for (CollisionCheckPlugin::Ptr plugin : getPlugins())
  {
    if (plugin && plugin->isCollisionCheckAvailable() && !plugin->isAccessible(s))
      return false;
  }
  return true;
}

bool WorldModel::isAccessible(const State& next, const State& current) const
{
  for (CollisionCheckPlugin::Ptr plugin : getPlugins())
  {
    if (plugin && plugin->isCollisionCheckAvailable() && !plugin->isAccessible(next, current))
      return false;
  }
  return true;
}

void WorldModel::useTerrainModel(bool enabled)
{
  use_terrain_model_ = enabled;
}

bool WorldModel::isTerrainModelAvailable() const
{
  return terrain_model_ && terrain_model_->isTerrainModelAvailable();
}

TerrainModelPlugin::ConstPtr WorldModel::getTerrainModel() const
{
  return terrain_model_;
}

bool WorldModel::getHeight(double x, double y, double& height) const
{
  if (!use_terrain_model_ || !isTerrainModelAvailable())
    return true;

  return terrain_model_->getHeight(x, y, height);
}

bool WorldModel::update3DData(State& s) const
{
  if (!use_terrain_model_ || !isTerrainModelAvailable())
    return true;

  return terrain_model_->update3DData(s);
}
}
