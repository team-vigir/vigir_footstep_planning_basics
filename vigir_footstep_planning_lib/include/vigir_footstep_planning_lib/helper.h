//=================================================================================================
// Copyright (c) 2017, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_LIB_HELPER_H__
#define VIGIR_FOOTSTEP_PLANNING_LIB_HELPER_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/OccupancyGrid.h>

#include <vigir_generic_params/generic_params_msgs.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>



namespace vigir_footstep_planning
{
template <typename ActionSpec>
class SimpleActionClient
    : public actionlib::SimpleActionClient<ActionSpec>
{
public:
  // typedefs
  typedef boost::shared_ptr<SimpleActionClient> Ptr;
  typedef boost::shared_ptr<const SimpleActionClient> ConstPtr;

  SimpleActionClient(ros::NodeHandle nh, std::string name, bool spin_thread = true)
    : actionlib::SimpleActionClient<ActionSpec>(nh, name, spin_thread)
  {
  }

  static Ptr create(ros::NodeHandle nh, std::string name, bool spin_thread = true)
  {
    return Ptr(new SimpleActionClient<ActionSpec>(nh, name, spin_thread));
  }
};

template <typename ActionSpec>
class SimpleActionServer
  : public actionlib::SimpleActionServer<ActionSpec>
{
public:
  // typedefs
  typedef boost::shared_ptr<SimpleActionServer<ActionSpec> > Ptr;
  typedef boost::shared_ptr<const SimpleActionServer<ActionSpec> > ConstPtr;
  typedef boost::function<void ()> ExecuteCallback;
  typedef boost::function<void ()> PreemptCallback;

  SimpleActionServer(ros::NodeHandle nh, std::string name, bool auto_start)
    : actionlib::SimpleActionServer<ActionSpec>(nh, name, auto_start)
  {
  }

  template <typename S> void finish(const S& result)
  {
    if (hasError(result.status))
      this->setAborted(result, toString(result.status));
    else
      this->setSucceeded(result, toString(result.status));
  }

  void preempt()
  {
    actionlib::SimpleActionServer<ActionSpec>::setPreempted();
  }

  static Ptr create(ros::NodeHandle nh, std::string name, bool auto_start, ExecuteCallback execute_cb, PreemptCallback preempt_cb = PreemptCallback())
  {
    Ptr as(new SimpleActionServer<ActionSpec>(nh, name, false));

    as->registerGoalCallback(execute_cb);

    if (!preempt_cb.empty())
      preempt_cb = boost::bind(&SimpleActionServer::preempt, as.get());
    as->registerPreemptCallback(preempt_cb);

    if (auto_start)
      as->start();

    return as;
  }
};

msgs::ErrorStatus determineStartFeetPose(msgs::Feet& start_feet, ros::ServiceClient& generate_feet_pose_client, const std_msgs::Header& header);
msgs::ErrorStatus determineGoalFeetPose(msgs::Feet& goal_feet, ros::ServiceClient& generate_feet_pose_client, const geometry_msgs::PoseStamped& goal_pose);

msgs::ErrorStatus transform(msgs::Foot& foot, ros::ServiceClient& transform_foot_pose_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::Feet& feet, ros::ServiceClient& transform_feet_poses_client, const std::string& target_frame);
msgs::ErrorStatus transform(msgs::StepPlan& step_plan, ros::ServiceClient& transform_step_plan_client, const std::string& target_frame);

template <typename T>
msgs::ErrorStatus transformToPlannerFrame(T& p, ros::ServiceClient& foot_pose_transformer_client)
{
  return transform(p, foot_pose_transformer_client, "planner");
}

template <typename T>
msgs::ErrorStatus transformToRobotFrame(T& p, ros::ServiceClient& foot_pose_transformer_client)
{
  return transform(p, foot_pose_transformer_client, "robot");
}



// loading of common parameters
bool getXYZ(ros::NodeHandle& nh, const std::string name, geometry_msgs::Vector3& val);
bool getRPY(ros::NodeHandle& nh, const std::string name, geometry_msgs::Vector3& val);

bool getFootSize(ros::NodeHandle& nh, geometry_msgs::Vector3& foot_size);
bool getUpperBodySize(ros::NodeHandle& nh, geometry_msgs::Vector3& upper_body_size);
bool getUpperBodyOriginShift(ros::NodeHandle& nh, geometry_msgs::Vector3& upper_body_origin_shift);



bool getGridMapCoords(const nav_msgs::OccupancyGrid& map, double x, double y, int& map_x, int& map_y);
bool getGridMapIndex(const nav_msgs::OccupancyGrid& map, double x, double y, int& idx);



inline std::string& strip(std::string& s, const char c) { return vigir_generic_params::strip(s, c); }
inline std::string strip_const(const std::string& s, const char c) { return vigir_generic_params::strip_const(s, c); }
}

#endif
