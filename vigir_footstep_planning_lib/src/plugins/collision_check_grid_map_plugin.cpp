#include <vigir_footstep_planning_lib/plugins/collision_check_grid_map_plugin.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{

CollisionCheckGridMapPlugin::CollisionCheckGridMapPlugin(const std::string& name, const vigir_generic_params::ParameterSet& params)
  : CollisionCheckPlugin(name, params)
  , occ_thresh(70)
{
}

CollisionCheckGridMapPlugin::CollisionCheckGridMapPlugin(const std::string& name)
  : CollisionCheckPlugin(name)
  , occ_thresh(70)
{
}

CollisionCheckGridMapPlugin::CollisionCheckGridMapPlugin()
  : CollisionCheckGridMapPlugin("collision_check_grid_map_plugin")
{
}

bool CollisionCheckGridMapPlugin::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!CollisionCheckPlugin::initialize(nh, params))
    return false;

  std::string topic;
  getPluginParam("grid_map_topic", topic, std::string("/grid_map"));
  occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &CollisionCheckGridMapPlugin::mapCallback, this);

  getPluginParam("occupancy_threshold", (int&) occ_thresh, (int&) occ_thresh, true);

  return true;
}

void CollisionCheckGridMapPlugin::reset()
{
  CollisionCheckPlugin::reset();

  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex);
  occupancy_grid_map.reset();
}

bool CollisionCheckGridMapPlugin::isCollisionCheckAvailable() const
{
  return CollisionCheckPlugin::isCollisionCheckAvailable() && occupancy_grid_map != nullptr;
}

bool CollisionCheckGridMapPlugin::isAccessible(const State& s) const
{
  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex);

  if (!occupancy_grid_map)
  {
    ROS_ERROR_THROTTLE(10, "[CollisionCheckGridMapPlugin] No grid map available yet.");
    return true;
  }

  double x = s.getX();
  double y = s.getY();
  int idx = 0;

  if (getGridMapIndex(*occupancy_grid_map, x, y, idx))
    return occupancy_grid_map->data.at(idx) <= occ_thresh;

  return false;
}

bool CollisionCheckGridMapPlugin::isAccessible(const State& next, const State& /*current*/) const
{
  return isAccessible(next);
}

void CollisionCheckGridMapPlugin::setOccupancyThreshold(unsigned char thresh)
{
  occ_thresh = static_cast<int8_t>(thresh);
}

void CollisionCheckGridMapPlugin::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex);
  this->occupancy_grid_map = occupancy_grid_map;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::CollisionCheckGridMapPlugin, vigir_footstep_planning::CollisionCheckPlugin)
