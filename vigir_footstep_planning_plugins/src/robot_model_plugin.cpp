#include <vigir_footstep_planning_plugins/robot_model_plugin.h>

namespace vigir_footstep_planning
{
using namespace vigir_generic_params;

RobotModelPlugin::RobotModelPlugin()
  : Plugin("robot_model")
{
}

bool RobotModelPlugin::initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params)
{
  if (!Plugin::initialize(nh, params))
    return false;

  // get foot dimensions
  vigir_footstep_planning::getFootSize(nh, foot_size_);

  // get upper body dimensions
  vigir_footstep_planning::getUpperBodySize(nh, upper_body_size_);
  vigir_footstep_planning::getUpperBodyOriginShift(nh, upper_body_origin_shift_);

  return true;
}

bool RobotModelPlugin::isUnique() const
{
  return true;
}

const geometry_msgs::Vector3& RobotModelPlugin::getFootSize() const
{
  return foot_size_;
}

const geometry_msgs::Vector3& RobotModelPlugin::getUpperBodySize() const
{
  return upper_body_size_;
}

const geometry_msgs::Vector3& RobotModelPlugin::getUpperBodyOriginShift() const
{
  return upper_body_origin_shift_;
}
}
