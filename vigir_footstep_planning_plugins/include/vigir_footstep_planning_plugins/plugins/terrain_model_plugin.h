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

#ifndef VIGIR_FOOTSTEP_PLANNING_PLUGINS_TERRAIN_MODEL_PLUGIN_H__
#define VIGIR_FOOTSTEP_PLANNING_PLUGINS_TERRAIN_MODEL_PLUGIN_H__

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <geometry_msgs/Pose.h>

#include <vigir_footstep_planning_plugins/plugins/collision_check_plugin.h>



namespace vigir_footstep_planning
{
class TerrainModelPlugin
  : public CollisionCheckPlugin
{
public:
  // typedefs
  typedef boost::shared_ptr<TerrainModelPlugin> Ptr;
  typedef boost::shared_ptr<const TerrainModelPlugin> ConstPtr;

  TerrainModelPlugin(const std::string& name);

  bool isUnique() const final;

  virtual bool isTerrainModelAvailable() const = 0;

  virtual double getResolution() const = 0;

  virtual bool getPointWithNormal(const pcl::PointNormal& p_search, pcl::PointNormal& p_result) const;
  virtual bool getHeight(double x, double y, double& height) const;
  virtual bool getFootContactSupport(const geometry_msgs::Pose& p, double& support, pcl::PointCloud<pcl::PointXYZI>::Ptr checked_positions = pcl::PointCloud<pcl::PointXYZI>::Ptr()) const;

  virtual bool update3DData(geometry_msgs::Pose& p) const = 0;
  virtual bool update3DData(State& s) const = 0;
};
}

#endif
