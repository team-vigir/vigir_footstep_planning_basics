#include <vigir_footstep_planning_lib/plugins/terrain_model_plugin.h>

namespace vigir_footstep_planning
{
TerrainModelPlugin::TerrainModelPlugin(const std::string& name, const vigir_generic_params::ParameterSet& params)
  : CollisionCheckPlugin(name, "terrain_model_plugin", params)
{
}

TerrainModelPlugin::TerrainModelPlugin(const std::string& name)
  : CollisionCheckPlugin(name, "terrain_model_plugin")
{
}

bool TerrainModelPlugin::isUnique() const
{
  return true;
}

bool TerrainModelPlugin::getPointWithNormal(const pcl::PointNormal& /*p_search*/, pcl::PointNormal& /*p_result*/) const
{
  return false;
}

bool TerrainModelPlugin::getHeight(double /*x*/, double /*y*/, double& /*height*/) const
{
  return false;
}

bool TerrainModelPlugin::getFootContactSupport(const geometry_msgs::Pose& /*s*/, double& /*support*/, pcl::PointCloud<pcl::PointXYZI>::Ptr /*checked_positions*/) const
{
  return false;
}
}
