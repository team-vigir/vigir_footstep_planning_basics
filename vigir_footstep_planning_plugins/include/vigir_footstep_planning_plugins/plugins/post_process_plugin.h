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

#ifndef VIGIR_FOOTSTEP_PLANNING_PLUGINS_POST_PROCESS_PLUGIN_H__
#define VIGIR_FOOTSTEP_PLANNING_PLUGINS_POST_PROCESS_PLUGIN_H__

#include <ros/ros.h>

#include <vigir_pluginlib/plugin.h>

#include <vigir_footstep_planning_lib/modeling/state.h>



namespace vigir_footstep_planning
{
class PostProcessPlugin
  : public vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef boost::shared_ptr<PostProcessPlugin> Ptr;
  typedef boost::shared_ptr<const PostProcessPlugin> ConstPtr;

  PostProcessPlugin(const std::string& name);

  /**
   * @brief Resets the plugin to initial state.
   */
  virtual void reset() {}

  bool isUnique() const final;

  /// Post-Processing directly after state generation
  virtual void postProcessStepForward(const State& left, const State& right, State& swing) const;
  virtual void postProcessStepBackward(const State& left, const State& right, State& swing) const;

  /// Post-Processing after footstep planning was done
  virtual void postProcessStep(const msgs::Step& left, const msgs::Step& right, msgs::Step& swing, msgs::StepPlan& step_plan) const;
  virtual void postProcess(msgs::StepPlan step_plan) const;
};
}

#endif
