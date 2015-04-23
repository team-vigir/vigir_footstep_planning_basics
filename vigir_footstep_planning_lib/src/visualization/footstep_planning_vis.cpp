#include <vigir_footstep_planning_lib/visualization/footstep_planning_vis.h>

namespace vigir_footstep_planning
{
namespace vis
{
void publishFeet(ros::Publisher& pub, const msgs::Feet& feet, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color)
{
  visualization_msgs::MarkerArray marker_array;
  msgs::feetToFootMarkerArray(feet, foot_size, color, marker_array);
  pub.publish(marker_array);
}

void publishStart(ros::Publisher& pub, const msgs::Feet& feet, const geometry_msgs::Vector3& foot_size)
{
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.7;
  color.b = 0.0;
  color.a = 0.6;
  publishFeet(pub, feet, foot_size, color);
}

void publishGoal(ros::Publisher& pub, const msgs::Feet& feet, const geometry_msgs::Vector3& foot_size)
{
  std_msgs::ColorRGBA color;
  color.r = 0.7;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 0.6;
  publishFeet(pub, feet, foot_size, color);
}

void publishStepPlan(ros::Publisher& pub, const msgs::StepPlan& step_plan, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& last_step_plan_vis, bool add_step_index)
{
  visualization_msgs::MarkerArray step_plan_vis;
  msgs::stepPlanToFootMarkerArray(step_plan, foot_size, step_plan_vis, add_step_index);

  // resize marker array
  if (last_step_plan_vis.markers.size() < step_plan_vis.markers.size())
    last_step_plan_vis.markers.resize(step_plan_vis.markers.size());

  // overwrite old markers
  for (size_t i = 0; i < step_plan_vis.markers.size(); i++)
    last_step_plan_vis.markers[i] = step_plan_vis.markers[i];

  // set needless markers to be deleted
  for (size_t i = step_plan_vis.markers.size(); i < last_step_plan_vis.markers.size(); i++)
    last_step_plan_vis.markers[i].action = visualization_msgs::Marker::DELETE;

  // finally publish new markers
  pub.publish(last_step_plan_vis);

  // delete old markers from array
  last_step_plan_vis.markers.resize(step_plan_vis.markers.size());
}

void publishUpperBody(ros::Publisher& pub, const msgs::StepPlan& step_plan, const geometry_msgs::Vector3& upper_body_size, const geometry_msgs::Vector3& upper_body_origin_shift, visualization_msgs::MarkerArray& last_upper_body_vis, bool add_step_index)
{
  visualization_msgs::MarkerArray upper_body_vis;
  msgs::stepPlanToUpperBodyMarkerArray(step_plan, upper_body_size, upper_body_origin_shift, upper_body_vis, add_step_index);

  // resize marker array
  if (last_upper_body_vis.markers.size() < upper_body_vis.markers.size())
    last_upper_body_vis.markers.resize(upper_body_vis.markers.size());

  // overwrite old markers
  for (size_t i = 0; i < upper_body_vis.markers.size(); i++)
    last_upper_body_vis.markers[i] = upper_body_vis.markers[i];

  // set needless markers to be deleted
  for (size_t i = upper_body_vis.markers.size(); i < last_upper_body_vis.markers.size(); i++)
    last_upper_body_vis.markers[i].action = visualization_msgs::Marker::DELETE;

  // finally publish new markers
  pub.publish(last_upper_body_vis);

  // delete old markers from array
  last_upper_body_vis.markers.resize(upper_body_vis.markers.size());
}

void publishPath(ros::Publisher& pub, const msgs::StepPlan& step_plan)
{
  nav_msgs::Path path;
  msgs::stepPlanToPath(step_plan, path);
  pub.publish(path);
}

void clearFeet(ros::Publisher& pub, const std_msgs::Header& header)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  marker.header = header;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::DELETE;

  marker.id = 0;
  marker_array.markers.push_back(marker);
  marker.id++;
  marker_array.markers.push_back(marker);

  pub.publish(marker_array);
}

void clearMarkerArray(ros::Publisher& pub, visualization_msgs::MarkerArray& last_step_plan_vis)
{
  for (std::vector<visualization_msgs::Marker>::iterator itr = last_step_plan_vis.markers.begin(); itr != last_step_plan_vis.markers.end(); itr++)
    itr->action = visualization_msgs::Marker::DELETE;
  pub.publish(last_step_plan_vis);
}

void clearPath(ros::Publisher& pub, const std_msgs::Header& header)
{
  nav_msgs::Path path;
  path.header = header;
  pub.publish(path);
}

void publishRecentlyVistedSteps(ros::Publisher& pub, const std::vector<msgs::Step>& recently_visited_steps, const std_msgs::Header& header)
{
  // populate point cloud
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  point_cloud.resize(recently_visited_steps.size());

  for (unsigned long i = 0; i < recently_visited_steps.size(); i++)
  {
    pcl::PointXYZ& point = point_cloud.points[i];
    copyPosition(recently_visited_steps[i].foot.pose.position, point);
  }

  // publish as point cloud msg
  sensor_msgs::PointCloud2 point_cloud_msgs;
  pcl::toROSMsg(point_cloud, point_cloud_msgs);
  point_cloud_msgs.header = header;
  pub.publish(point_cloud_msgs);
}

void publishVistedSteps(ros::Publisher& pub, const std::vector<msgs::Step>& visited_steps, const std_msgs::Header& header)
{
  // populate point cloud
  pcl::PointCloud<pcl::PointXYZI> point_cloud;
  point_cloud.resize(visited_steps.size());

  ros::Time current_time = ros::Time::now();
  unsigned long i = 0;
  for (std::vector<msgs::Step>::const_iterator itr = visited_steps.begin(); itr != visited_steps.end(); itr++, i++)
  {
    pcl::PointXYZI& point = point_cloud.points[i];
    copyPosition(itr->foot.pose.position, point);
    point.intensity = std::min(5.0, (current_time - itr->header.stamp).toSec());
  }

  // publish as point cloud msg
  sensor_msgs::PointCloud2 point_cloud_msgs;
  pcl::toROSMsg(point_cloud, point_cloud_msgs);
  point_cloud_msgs.header = header;
  pub.publish(point_cloud_msgs);
}

void publishVistedSteps(ros::Publisher& pub, const std::set<msgs::Step, StepMsgVisCompare>& visited_steps, const std_msgs::Header& header)
{
  std::vector<msgs::Step> vec;
  for (std::set<msgs::Step, StepMsgVisCompare>::const_iterator itr = visited_steps.begin(); itr != visited_steps.end(); itr++)
    vec.push_back(*itr);
  publishVistedSteps(pub, vec, header);
}
}
}
