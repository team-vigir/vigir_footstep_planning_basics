#include <vigir_footstep_planning_lib/plugins/collision_check_grid_map_plugin.h>

namespace vigir_footstep_planning
{

CollisionCheckGridMapPlugin::CollisionCheckGridMapPlugin(const std::string& name, const ParameterSet& params, unsigned int collision_check_flag, ros::NodeHandle& nh, const std::string& topic, unsigned char thresh)
  : CollisionCheckPlugin(name, "collision_check_grid_map_plugin", params, collision_check_flag)
  , occ_thresh(static_cast<int8_t>(thresh))
{
  occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &CollisionCheckGridMapPlugin::mapCallback, this);
}

CollisionCheckGridMapPlugin::CollisionCheckGridMapPlugin(const std::string& name, unsigned int collision_check_flag, ros::NodeHandle& nh, const std::string& topic, unsigned char thresh)
  : CollisionCheckPlugin(name, "collision_check_grid_map_plugin", collision_check_flag)
  , occ_thresh(static_cast<int8_t>(thresh))
{
  occupancy_grid_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &CollisionCheckGridMapPlugin::mapCallback, this);
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
