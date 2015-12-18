#include <vigir_footstep_planning_lib/helper.h>

namespace vigir_footstep_planning
{
msgs::ErrorStatus determineStartFeetPose(msgs::Feet& start_feet, ros::ServiceClient& generate_feet_pose_client, const std_msgs::Header& header)
{
  msgs::ErrorStatus status;

  // get start feet pose
  msgs::GenerateFeetPoseService feet_pose_service;
  feet_pose_service.request.request.header = header;
  feet_pose_service.request.request.flags = msgs::FeetPoseRequest::FLAG_CURRENT | msgs::FeetPoseRequest::FLAG_3D;

  if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "determineStartFeetPose", "Can't call 'FeetPoseGenerator'!");

  // check result
  if (hasError(feet_pose_service.response.status))
  {
    //status += feet_pose_service.response.status;
    status += ErrorStatusWarning(msgs::ErrorStatus::WARN_UNKNOWN, "determineStartFeetPose", "Can't obtain start feet pose; defaulting to origin.");

    feet_pose_service.request.request.pose = geometry_msgs::Pose();
    feet_pose_service.request.request.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    feet_pose_service.request.request.flags = msgs::FeetPoseRequest::FLAG_3D;

    if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
      return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "determineStartFeetPose", "Can't call 'FeetPoseGenerator'!");

    if (hasError(feet_pose_service.response.status))
    {
      status += feet_pose_service.response.status;
      return status;
    }
  }
  else if (hasWarning(feet_pose_service.response.status))
    status += feet_pose_service.response.status;

  start_feet = feet_pose_service.response.feet;
  return status;
}

msgs::ErrorStatus determineGoalFeetPose(msgs::Feet& goal_feet, ros::ServiceClient& generate_feet_pose_client, const geometry_msgs::PoseStamped& goal_pose)
{
  msgs::GenerateFeetPoseService feet_pose_service;
  feet_pose_service.request.request.header = goal_pose.header;
  feet_pose_service.request.request.pose = goal_pose.pose;
  feet_pose_service.request.request.flags = msgs::FeetPoseRequest::FLAG_3D;

  if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "determineStartFeetPose", "Can't call 'FeetPoseGenerator'!");

  goal_feet = feet_pose_service.response.feet;

  return feet_pose_service.response.status;
}

msgs::ErrorStatus transform(msgs::Foot& foot, ros::ServiceClient& transform_foot_pose_client, const std::string& target_frame)
{
  msgs::TransformFootPoseService transform_service;
  transform_service.request.foot = foot;
  transform_service.request.target_frame.data = target_frame;

  if (!transform_foot_pose_client.call(transform_service.request, transform_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "transformToPlannerFrame", "Can't call 'FootPoseTransformer' for foot pose transform!");

  foot = transform_service.response.foot;

  return transform_service.response.status;
}

msgs::ErrorStatus transform(msgs::Feet& feet, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame)
{
  msgs::TransformFeetPosesService transform_service;
  transform_service.request.feet = feet;
  transform_service.request.target_frame.data = target_frame;

  if (!transform_feet_poses_client.call(transform_service.request, transform_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "transformToPlannerFrame", "Can't call 'FootPoseTransformer' for feet poses transform!");

  feet = transform_service.response.feet;

  return transform_service.response.status;
}

msgs::ErrorStatus transform(msgs::StepPlan& step_plan, ros::ServiceClient& transform_step_plan_client, const std::string& target_frame)
{
  msgs::TransformStepPlanService transform_service;
  transform_service.request.step_plan = step_plan;
  transform_service.request.target_frame.data = target_frame;

  if (!transform_step_plan_client.call(transform_service.request, transform_service.response))
    return ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "transformToPlannerFrame", "Can't call 'FootPoseTransformer' for step plan transform!");

  step_plan = transform_service.response.step_plan;

  return transform_service.response.status;
}



bool getXYZ(ros::NodeHandle& nh, const std::string name, geometry_msgs::Vector3& val)
{
  if (!nh.hasParam(name+"/x") || !nh.hasParam(name+"/y") || !nh.hasParam(name+"/z"))
  {
    ROS_ERROR("Couldn't retrieve parameter '%s' as Vector3", (nh.getNamespace()+"/"+name).c_str());
    return false;
  }

  nh.getParam(name+"/x", val.x);
  nh.getParam(name+"/y", val.y);
  nh.getParam(name+"/z", val.z);
  return true;
}

bool getRPY(ros::NodeHandle& nh, const std::string name, geometry_msgs::Vector3& val)
{
  if (!nh.hasParam(name+"/roll") || !nh.hasParam(name+"/pitch") || !nh.hasParam(name+"/yaw"))
  {
    ROS_ERROR("Couldn't retrieve parameter '%s' as RPY orienation", (nh.getNamespace()+"/"+name).c_str());
    return false;
  }

  nh.getParam(name+"/roll", val.x);
  nh.getParam(name+"/pitch", val.y);
  nh.getParam(name+"/yaw", val.z);
  return true;
}

bool getFootSize(ros::NodeHandle& nh, geometry_msgs::Vector3& foot_size)
{
  return getXYZ(nh, "foot/size", foot_size);
}

bool getUpperBodySize(ros::NodeHandle& nh, geometry_msgs::Vector3& upper_body_size)
{
  return getXYZ(nh, "upper_body/size", upper_body_size);
}

bool getUpperBodyOriginShift(ros::NodeHandle& nh, geometry_msgs::Vector3& upper_body_origin_shift)
{
  return getXYZ(nh, "upper_body/origin_shift", upper_body_origin_shift);
}



bool getGridMapCoords(const nav_msgs::OccupancyGrid& map, double x, double y, int& map_x, int& map_y)
{
  map_x = round((x-map.info.origin.position.x)/map.info.resolution);
  map_y = round((y-map.info.origin.position.y)/map.info.resolution);

  if (map_x < 0 || (int) map.info.width  <= map_x ||
      map_y < 0 || (int) map.info.height <= map_y)
    return false;

  return true;
}

bool getGridMapIndex(const nav_msgs::OccupancyGrid& map, double x, double y, int& idx)
{
  int map_x, map_y;

  if (!getGridMapCoords(map, x, y, map_x, map_y))
    return false;
  else
    idx = map_x + map_y * map.info.width;

  return true;
}
}
