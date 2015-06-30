#include <vigir_footstep_planning_lib/plugins/robot_model_plugin.h>

namespace vigir_footstep_planning
{
using namespace vigir_generic_params;

RobotModelPlugin::RobotModelPlugin(const vigir_generic_params::ParameterSet& params, ros::NodeHandle &nh)
  : vigir_pluginlib::Plugin("robot_model", "robot_model_plugin", params)
{
  // get foot dimensions
  vigir_footstep_planning::getFootSize(nh, foot_size);

  // get upper body dimensions
  vigir_footstep_planning::getUpperBodySize(nh, upper_body_size);
  vigir_footstep_planning::getUpperBodyOriginShift(nh, upper_body_origin_shift);
}

RobotModelPlugin::RobotModelPlugin(ros::NodeHandle &nh)
  : vigir_pluginlib::Plugin("robot_model", "robot_model_plugin")
{
  // get foot dimensions
  vigir_footstep_planning::getFootSize(nh, foot_size);

  // get upper body dimensions
  vigir_footstep_planning::getUpperBodySize(nh, upper_body_size);
  vigir_footstep_planning::getUpperBodyOriginShift(nh, upper_body_origin_shift);
}

bool RobotModelPlugin::isUnique() const
{
  return true;
}

const geometry_msgs::Vector3& RobotModelPlugin::getFootSize() const
{
  return foot_size;
}

const geometry_msgs::Vector3& RobotModelPlugin::getUpperBodySize() const
{
  return upper_body_size;
}

const geometry_msgs::Vector3& RobotModelPlugin::getUpperBodyOriginShift() const
{
  return upper_body_origin_shift;
}
}
