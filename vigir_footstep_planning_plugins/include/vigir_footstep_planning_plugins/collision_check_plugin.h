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

#ifndef VIGIR_FOOTSTEP_PLANNING_LIB_COLLISION_CHECK_PLUGIN_H__
#define VIGIR_FOOTSTEP_PLANNING_LIB_COLLISION_CHECK_PLUGIN_H__

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <vigir_pluginlib/plugin.h>

#include <vigir_footstep_planning_lib/modeling/state.h>



namespace vigir_footstep_planning
{
class CollisionCheckPlugin
  : public vigir_pluginlib::Plugin
{
public:
  enum
  {
    FOOT                  = 1,
    UPPER_BODY            = 2,
    FOOT_CONTACT_SUPPORT  = 4
  };

  // typedefs
  typedef boost::shared_ptr<CollisionCheckPlugin> Ptr;
  typedef boost::shared_ptr<const CollisionCheckPlugin> ConstPtr;

  CollisionCheckPlugin(const std::string& name);

  bool initialize(const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet()) override;

  bool loadParams(const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet()) override;

  virtual void reset();

  bool isUnique() const override;
  virtual bool isCollisionCheckAvailable() const;

  virtual bool isAccessible(const State& s) const = 0;
  virtual bool isAccessible(const State& next, const State& current) const = 0;

private:
  bool collision_check_enabled_;
  unsigned int collision_check_flag_;
};
}

#endif
