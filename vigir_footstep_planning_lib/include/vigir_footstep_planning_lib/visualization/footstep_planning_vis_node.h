//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_FOOTSTEP_PLANNING_VIS_NODE_H__
#define VIGIR_FOOTSTEP_PLANNING_VIS_NODE_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_footstep_planning_lib/visualization/footstep_planning_vis.h>



namespace vigir_footstep_planning
{
class FootstepPlanningVisNode
{
public:
  FootstepPlanningVisNode();
  virtual ~FootstepPlanningVisNode();

protected:
  void stepPlanRequestVisCallback(const msgs::StepPlanRequestConstPtr& step_plan_request);
  void stepPlanVisCallback(const msgs::StepPlanConstPtr& step_plan);
  void planningFeedbackCallback(const msgs::PlanningFeedbackConstPtr& planning_feedback);

  void clearVisualization(const std_msgs::Header& header);

  // subscriber
  ros::Subscriber step_plan_request_vis_sub;
  ros::Subscriber step_plan_vis_sub;
  ros::Subscriber planning_feedback_sub;

  // publisher
  ros::Publisher step_plan_vis_pub;
  ros::Publisher upper_body_vis_pub;
  ros::Publisher step_plan_path_pub;
  ros::Publisher start_feet_pose_pub;
  ros::Publisher goal_feet_pose_pub;
  ros::Publisher visited_steps_pub;
  ros::Publisher total_visited_steps_pub;
  ros::Publisher current_step_plan_pub;

  // service clients
  ros::ServiceClient transform_foot_pose_client;
  ros::ServiceClient transform_feet_poses_client;
  ros::ServiceClient transform_step_plan_client;

  // robot parameters
  geometry_msgs::Vector3 foot_size;
  geometry_msgs::Vector3 upper_body_size;
  geometry_msgs::Vector3 upper_body_origin_shift;

  // visualization helper
  visualization_msgs::MarkerArray last_step_plan_vis;
  visualization_msgs::MarkerArray last_upper_body_vis;
  visualization_msgs::MarkerArray last_current_step_plan_vis;
  std::set<msgs::Step, vis::StepMsgVisCompare> total_visited_steps;
};
}

#endif
