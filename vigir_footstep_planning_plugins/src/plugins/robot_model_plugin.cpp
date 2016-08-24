#include <vigir_footstep_planning_plugins/plugins/robot_model_plugin.h>



namespace vigir_footstep_planning
{
using namespace vigir_generic_params;

RobotModelPlugin::RobotModelPlugin()
  : Plugin("robot_model")
{
}

bool RobotModelPlugin::initialize(const vigir_generic_params::ParameterSet& global_params)
{
  if (!Plugin::initialize(global_params))
    return false;

  // get foot dimensions
  vigir_footstep_planning::getFootSize(nh_, foot_size_);

  // get upper body dimensions
  vigir_footstep_planning::getUpperBodySize(nh_, upper_body_size_);
  vigir_footstep_planning::getUpperBodyOriginShift(nh_, upper_body_origin_shift_);

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
