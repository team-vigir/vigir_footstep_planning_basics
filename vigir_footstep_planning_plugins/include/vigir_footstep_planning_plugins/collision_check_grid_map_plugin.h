//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
// Based on http://wiki.ros.org/footstep_planner by Johannes Garimort and Armin Hornung
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

#ifndef VIGIR_FOOTSTEP_PLANNING_LIB_COLLISION_CHECK_GRID_MAP_PLUGIN_H__
#define VIGIR_FOOTSTEP_PLANNING_LIB_COLLISION_CHECK_GRID_MAP_PLUGIN_H__

#include <ros/ros.h>

#include <boost/thread/shared_mutex.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <vigir_footstep_planning_lib/helper.h>

#include <vigir_footstep_planning_plugins/collision_check_plugin.h>



namespace vigir_footstep_planning
{
class CollisionCheckGridMapPlugin
  : public CollisionCheckPlugin
{
public:
  CollisionCheckGridMapPlugin(const std::string& name = "collision_check_grid_map_plugin");

  bool initialize(ros::NodeHandle& nh, const vigir_generic_params::ParameterSet& params) override;

  void reset() override;

  bool isCollisionCheckAvailable() const override;

  bool isAccessible(const State& s) const override;
  bool isAccessible(const State& next, const State& current) const override;

  void setOccupancyThreshold(unsigned char thresh);

protected:
  virtual void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map);

  // subscribers
  ros::Subscriber occupancy_grid_map_sub;

  // mutex
  mutable boost::shared_mutex grid_map_shared_mutex;

  // pointer to last received grid map
  nav_msgs::OccupancyGridConstPtr occupancy_grid_map;

  // occupancy threshold
  int8_t occ_thresh;
};
}

#endif
