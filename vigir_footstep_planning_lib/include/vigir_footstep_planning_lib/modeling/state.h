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

#ifndef FOOTSTEP_PLANNER_STATE_H__
#define FOOTSTEP_PLANNER_STATE_H__

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_footstep_planning_lib/math.h>



namespace vigir_footstep_planning
{
/**
 * @brief A class representing the robot's pose (i.e. position and
 * orientation) in the (continuous) world view. More precisely a state
 * points to the robot's supporting leg.
 */
class State
{
public:
  State();
  State(double x, double y, double z, double roll, double pitch, double yaw, double swing_height, double step_duration, Leg leg);
  State(const geometry_msgs::Vector3& position, double roll, double pitch, double yaw, double swing_height, double step_duration, Leg leg);
  State(const geometry_msgs::Vector3& position, const geometry_msgs::Vector3& normal, double yaw, double swing_height, double step_duration, Leg leg);
  State(const geometry_msgs::Pose& pose, double swing_height, double step_duration, Leg leg);
  State(const tf::Transform& t, double swing_height, double step_duration, Leg leg);
  State(const msgs::Foot foot, double swing_height, double step_duration);
  State(const msgs::Step step);

  ~State();

  /**
   * @brief Compare two states on equality of x, y, theta, leg upon
   * a certain degree of float precision.
   */
  bool operator==(const State& s2) const;

  /**
   * @brief Inequality operator for two states (negates the equality
   * operator).
   */
  bool operator!=(const State& s2) const;

  void setX(double x) { ivPose.getOrigin().setX(x); }
  void setY(double y) { ivPose.getOrigin().setY(y); }
  void setZ(double z) { ivPose.getOrigin().setZ(z); }
  void setPosition(const geometry_msgs::Point& position);
  void setRoll(double roll) { setRPY(roll, ivPitch, ivYaw); }
  void setPitch(double pitch) { setRPY(ivRoll, pitch, ivYaw); }
  void setYaw(double yaw) { setRPY(ivRoll, ivPitch, yaw); }
  void setRPY(double roll, double pitch, double yaw);
  void setOrientation(const geometry_msgs::Quaternion& q);
  void setNormal(const geometry_msgs::Vector3 &normal);
  void setNormal(double x, double y, double z);
  void setSwingHeight(double swing_height) { ivSwingHeight = swing_height; }
  void setStepDuration(double step_duration) { ivStepDuration = step_duration; }
  void setLeg(Leg leg) { ivLeg = leg; }
  void setGroundContactSupport(double ground_contact_support) { ivGroundContactSupport = ground_contact_support; }
  void setCost(double cost) { this->cost = cost; }
  void setRisk(double risk) { this->risk = risk; }

  double getX() const { return ivPose.getOrigin().getX(); }
  double getY() const { return ivPose.getOrigin().getY(); }
  double getZ() const { return ivPose.getOrigin().getZ(); }
  double getRoll() const { return ivRoll; }
  double getPitch() const { return ivPitch; }
  double getYaw() const { return ivYaw; }
  const geometry_msgs::Vector3& getNormal() const { return ivNormal; }
  double getNormalX() const { return ivNormal.x; }
  double getNormalY() const { return ivNormal.y; }
  double getNormalZ() const { return ivNormal.z; }
  double getSwingHeight() const { return ivSwingHeight; }
  double getStepDuration() const { return ivStepDuration; }
  Leg getLeg() const { return ivLeg; }
  double getGroundContactSupport() const { return ivGroundContactSupport; }
  double getCost() { return cost; }
  double getRisk() { return risk; }


  const tf::Pose &getPose() const { return ivPose; }
  tf::Pose &getPose() { return ivPose; }
  void getFoot(msgs::Foot& foot) const;
  void getStep(msgs::Step& step) const;

private:
  void recomputeNormal();

  tf::Pose ivPose;

  /// The robot's orientation.
  double ivRoll;
  double ivPitch;
  double ivYaw;
  /// The normal of foot in world.
  geometry_msgs::Vector3 ivNormal;
  double ivSwingHeight;
  double ivStepDuration;
  /// The robot's supporting leg.
  Leg ivLeg;

  /// leg's knee nominal
  double ivKneeNominal;

  /// percentage of ground contact support (0.0 - 1.0 = 100%)
  double ivGroundContactSupport;

  double cost;
  double risk;
};
}
#endif
