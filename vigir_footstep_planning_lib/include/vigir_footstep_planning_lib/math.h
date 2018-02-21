//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_LIB_MATH_H__
#define VIGIR_FOOTSTEP_PLANNING_LIB_MATH_H__

#define DEBUG_HASH 0
#define DEBUG_TIME 0

#include <math.h>

#include <tf/tf.h>
#include <angles/angles.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>



namespace vigir_footstep_planning
{
static const double TWO_PI = 2 * M_PI;

static const double FLOAT_CMP_THR = 0.0001;

/// Used to scale continuous values in meter to discrete values in mm.
static const int cvMmScale = 1000;

enum Leg { RIGHT=0, LEFT=1, NOLEG=2 };


/**
 * @return Squared euclidean distance between two integer coordinates
 * (cells).
 */
inline double euclidean_distance_sq(int x1, int y1, int x2, int y2)
{
  // note: do *not* use pow() to square!
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}

/// @return Squared euclidean distance between two coordinates.
inline double euclidean_distance_sq(double x1, double y1, double x2, double y2)
{
  // note: do *not* use pow() to square!
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}

inline double euclidean_distance_sq(int x1, int y1, int z1, int x2, int y2, int z2)
{
  // note: do *not* use pow() to square!
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2);
}

/// @return Squared euclidean distance between two coordinates.
inline double euclidean_distance_sq(double x1, double y1, double z1, double x2, double y2, double z2)
{
  // note: do *not* use pow() to square!
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2);
}

/// @return Euclidean distance between two integer coordinates (cells).
inline double euclidean_distance(int x1, int y1, int x2, int y2)
{
  return sqrt(double(euclidean_distance_sq(x1, y1, x2, y2)));
}

/// @return Euclidean distance between two coordinates.
inline double euclidean_distance(double x1, double y1, double x2, double y2)
{
  return sqrt(euclidean_distance_sq(x1, y1, x2, y2));
}

/// @return Euclidean distance between two integer coordinates (cells).
inline double euclidean_distance(int x1, int y1, int z1, int x2, int y2, int z2)
{
  return sqrt(double(euclidean_distance_sq(x1, y1, z1, x2, y2, z2)));
}

/// @return Euclidean distance between two coordinates.
inline double euclidean_distance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return sqrt(euclidean_distance_sq(x1, y1, z1, x2, y2, z2));
}

inline double parabol(double x, double y, double a_inv, double b_inv)
{
  return x*x*a_inv + y*y*b_inv;
}

/// @return The distance of two neighbored cell.
inline double grid_cost(int x1, int y1, int x2, int y2, float cell_size)
{
  int x = abs(x1 - x2);
  int y = abs(y1 - y2);

  if (x + y > 1)
    return M_SQRT2 * cell_size;
  else
    return cell_size;
}


inline double pround(double x, double prec)
{
  return ::round(x/prec)*prec;
}

inline double pceil(double x, double prec)
{
  return ::ceil(x/prec)*prec;
}

inline double pfloor(double x, double prec)
{
  return ::floor(x/prec)*prec;
}


/// @brief Discretize a (continuous) value into cell size.
// TODO: check consistency for negative values
inline int disc_val(double length, double cell_size)
{
  //return int(floor((length / cell_size) + 0.5));
  return static_cast<int>(round(length / cell_size));
}


/**
 * @brief Calculates the continuous value for a length discretized in cell
 * size.
 */
// TODO: check consistency for negative values
inline double cont_val(int length, double cell_size)
{
  //return double(length * cell_size);
  return static_cast<double>(length) * cell_size;
}


/// @brief Discretize a (continuous) angle into a bin.
inline int angle_state_2_cell(double angle, double angle_bin_size)
{
//  double bin_size_half = M_PI / angle_bin_num;
//  return int(angles::normalize_angle_positive(angle + bin_size_half) /
//             TWO_PI * angle_bin_num);
  //return int(angles::normalize_angle_positive(angle + 0.5*angle_bin_size) / angle_bin_size);
  return disc_val(angle, angle_bin_size);
}


/// @brief Calculate the respective (continuous) angle for a bin.
inline double angle_cell_2_state(int angle, double angle_bin_size)
{
//  double bin_size = TWO_PI / angle_bin_num;
//  return angle * bin_size;
  //return double(angle * angle_bin_size);
  return cont_val(angle, angle_bin_size);
}


/**
 * @brief Discretize a (continuous) state value into a cell. (Should be
 * used to discretize a State to a PlanningState.)
 */
inline int state_2_cell(float value, float cell_size)
{
  //return value >= 0 ? int(value / cell_size) : int(value / cell_size) - 1;
  return disc_val(value, cell_size);
}


/**
 * @brief Calculate the respective (continuous) state value for a cell.
 * (Should be used to get a State from a discretized PlanningState.)
 */
inline double cell_2_state(int value, double cell_size)
{
  //return (double(value) + 0.5) * cell_size;
  return cont_val(value, cell_size);
}


/// @return The hash value of the key.
inline unsigned int int_hash(int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}


/**
 * @return The hash tag for a PlanningState (represented by x, y, theta and
 * leg). The component z may be ignored because it depends directly on x and y.
 * Also swing height and step duration will be uniquely selected for each x and y tupel.
 */
inline unsigned int calc_hash_tag(int x, int y, int theta, int leg, unsigned int hash_pred, unsigned int hash_succ, int max_hash_size)
{
  return int_hash((int_hash(x) << 3) + (int_hash(y) << 2) + (int_hash(theta) << 1) +
                  (int_hash(leg))) % max_hash_size;

//  return int_hash((int_hash(x) << 4) + (int_hash(y) << 3) + (int_hash(theta) << 2) +
//                  (int_hash(leg) << 1)/* + hash_pred + hash_succ*/) % max_hash_size;
}

/**
 * @brief Crossing number method to determine whether a point lies within a
 * polygon or not.
 * @param edges (x,y)-points defining the polygon.
 *
 * Check http://geomalgorithms.com/a03-_inclusion.html for further details.
 */
bool pointWithinPolygon(int x, int y, const std::vector<std::pair<int, int> >& edges);

void getDeltaStep(const msgs::Foot& current, const msgs::Foot& next, geometry_msgs::Pose& dstep);
void getDeltaStep(const tf::Pose& current, const tf::Pose& next, tf::Pose& dstep);

void quaternionToNormalYaw(const geometry_msgs::Quaternion& q, geometry_msgs::Vector3& n, double& yaw);
void quaternionToNormal(const geometry_msgs::Quaternion& q, geometry_msgs::Vector3& n);
void normalToQuaternion(const geometry_msgs::Vector3& n, double yaw, geometry_msgs::Quaternion& q);

void RPYToNormal(double r, double p, double y, geometry_msgs::Vector3& n);
void normalToRP(const geometry_msgs::Vector3& n, double yaw, double& r, double& p);
}

#endif
