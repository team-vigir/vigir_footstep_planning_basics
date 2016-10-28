#include <vigir_footstep_planning_plugins/plugins/collision_check_grid_map_plugin.h>



namespace vigir_footstep_planning
{
CollisionCheckGridMapPlugin::CollisionCheckGridMapPlugin(const std::string& name)
  : CollisionCheckPlugin(name)
  , occ_thresh_(70)
{
}

bool CollisionCheckGridMapPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!CollisionCheckPlugin::initialize(params))
    return false;

  std::string topic;
  getParam("grid_map_topic", topic, std::string("/grid_map"));
  occupancy_grid_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(topic, 1, &CollisionCheckGridMapPlugin::mapCallback, this);

  getParam("occupancy_threshold", (int&) occ_thresh_, (int&) occ_thresh_, true);

  return true;
}

void CollisionCheckGridMapPlugin::reset()
{
  CollisionCheckPlugin::reset();

  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);
  occupancy_grid_map_.reset();
}

bool CollisionCheckGridMapPlugin::isCollisionCheckAvailable() const
{
  return CollisionCheckPlugin::isCollisionCheckAvailable() && occupancy_grid_map_ != nullptr;
}

bool CollisionCheckGridMapPlugin::isAccessible(const State& s) const
{
  boost::shared_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);

  if (!occupancy_grid_map_)
  {
    ROS_ERROR_THROTTLE(10, "[CollisionCheckGridMapPlugin] No grid map available yet.");
    return true;
  }

  double x = s.getX();
  double y = s.getY();
  int idx = 0;

  if (getGridMapIndex(*occupancy_grid_map_, x, y, idx))
    return occupancy_grid_map_->data.at(idx) <= occ_thresh_;

  return false;
}

bool CollisionCheckGridMapPlugin::isAccessible(const State& next, const State& /*current*/) const
{
  return isAccessible(next);
}

void CollisionCheckGridMapPlugin::setOccupancyThreshold(unsigned char thresh)
{
  occ_thresh_ = static_cast<int8_t>(thresh);
}

void CollisionCheckGridMapPlugin::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map)
{
  boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);
  this->occupancy_grid_map_ = occupancy_grid_map;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::CollisionCheckGridMapPlugin, vigir_footstep_planning::CollisionCheckPlugin)
