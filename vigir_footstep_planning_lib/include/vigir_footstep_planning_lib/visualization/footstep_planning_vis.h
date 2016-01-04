//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_VIS_H__
#define VIGIR_FOOTSTEP_PLANNING_VIS_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/visualization.h>



namespace vigir_footstep_planning
{
namespace vis
{
struct StepMsgVisCompare
{
  bool operator() (const msgs::Step& lhs, const msgs::Step& rhs) const
  {
    double lhs_a[3];
    toArray(lhs, lhs_a);
    double rhs_a[3];
    toArray(rhs, rhs_a);

    // we can't visualize orientation, thus we ignore it
    for (size_t i = 0; i < 3; i++)
    {
      if (lhs_a[i] < rhs_a[i])
        return true;
      else if (lhs_a[i] > rhs_a[i])
        return false;
    }
    return false;
  }

  void toArray(const msgs::Step s, double (&a)[3]) const
  {
    a[0] = s.foot.pose.position.x;
    a[1] = s.foot.pose.position.y;
    a[2] = s.foot.pose.position.z;
  }
};

void publishFeet(ros::Publisher& pub, const msgs::Feet& feet, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color);
void publishStart(ros::Publisher& pub, const msgs::Feet& feet, const geometry_msgs::Vector3& foot_size);
void publishGoal(ros::Publisher& pub, const msgs::Feet& feet, const geometry_msgs::Vector3& foot_size);

void publishStepPlan(ros::Publisher& pub, const msgs::StepPlan& step_plan, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& last_step_plan_vis, bool add_step_index = true);
void publishUpperBody(ros::Publisher& pub, const msgs::StepPlan& step_plan, const geometry_msgs::Vector3& upper_body_size, const geometry_msgs::Vector3& upper_body_origin_shift, visualization_msgs::MarkerArray& last_upper_body_vis, bool add_step_index = true);
void publishPath(ros::Publisher& pub, const msgs::StepPlan& step_plan);

void clearFeet(ros::Publisher& pub, const std_msgs::Header& header);
void clearMarkerArray(ros::Publisher& pub, visualization_msgs::MarkerArray& last_step_plan_vis);
void clearPath(ros::Publisher& pub, const std_msgs::Header& header);

void publishRecentlyVistedSteps(ros::Publisher& pub, const std::vector<msgs::Step>& recently_visited_steps, const std_msgs::Header& header);
void publishVistedSteps(ros::Publisher& pub, const std::vector<msgs::Step>& visited_steps, const std_msgs::Header& header);
void publishVistedSteps(ros::Publisher& pub, const std::set<msgs::Step, StepMsgVisCompare>& visited_steps, const std_msgs::Header& header);
}
}

#endif
