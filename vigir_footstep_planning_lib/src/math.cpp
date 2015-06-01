#include <vigir_footstep_planning_lib/math.h>

namespace vigir_footstep_planning
{
bool pointWithinPolygon(int x, int y, const std::vector<std::pair<int, int> >& edges)
{
  int cn = 0;

  // loop through all edges of the polygon
  for(unsigned int i = 0; i < edges.size(); ++i)
  {
    unsigned int i_plus = (i + 1) % edges.size();
    if ((edges[i].second <= y && edges[i_plus].second > y) ||
        (edges[i].second > y && edges[i_plus].second <= y))
    {
      if (fabs(edges[i_plus].second - edges[i].second) > FLOAT_CMP_THR)
      {
        float vt = (float)(y - edges[i].second) / (edges[i_plus].second - edges[i].second);
        if (x < edges[i].first + vt * (edges[i_plus].first - edges[i].first))
          cn++;
      }
    }
  }
  return cn & 1;
}

void getDeltaStep(const msgs::Foot& current, const msgs::Foot& next, geometry_msgs::Pose& dstep)
{
  tf::Pose current_tf;
  tf::poseMsgToTF(current.pose, current_tf);

  tf::Pose next_tf;
  tf::poseMsgToTF(next.pose, next_tf);

  tf::Pose dstep_tf;
  getDeltaStep(current_tf, next_tf, dstep_tf);
  tf::poseTFToMsg(dstep_tf, dstep);

  // adjust for the left foot
  if (current.foot_index != next.foot_index && current.foot_index == msgs::Foot::LEFT)
  {
    dstep.position.y = -dstep.position.y;

    double r, p, y;
    dstep_tf.getBasis().getRPY(r, p, y);
    dstep.orientation = tf::createQuaternionMsgFromRollPitchYaw(-r, p, -y);

//    double dyaw = angles::shortest_angular_distance(current.getYaw(), next.getYaw());
//    dy = -dy;
//    dyaw = -dyaw;
  }
}

void getDeltaStep(const tf::Pose& current, const tf::Pose& next, tf::Pose& dstep)
{
  // reconstruct step primitive (current -> next, must be mirrored if current is left foot)
  dstep = current.inverse() * next;
}

void quaternionToNormalYaw(const geometry_msgs::Quaternion& q, geometry_msgs::Vector3& n, double& yaw)
{
  tf::Quaternion q_tf;
  tf::quaternionMsgToTF(q, q_tf);

  double r, p, y;
  tf::Matrix3x3(q_tf).getRPY(r, p, y);

  RPYToNormal(r, p, y, n);

  yaw = y;
}

void quaternionToNormal(const geometry_msgs::Quaternion& q, geometry_msgs::Vector3& n)
{
  tf::Quaternion q_tf;
  tf::quaternionMsgToTF(q, q_tf);

  double r, p, y;
  tf::Matrix3x3(q_tf).getRPY(r, p, y);

  RPYToNormal(r, p, y, n);
}

void normalToQuaternion(const geometry_msgs::Vector3& n, double yaw, geometry_msgs::Quaternion& q)
{
  double r, p;
  normalToRP(n, yaw, r, p);
  q = tf::createQuaternionMsgFromRollPitchYaw(r, p, yaw);
}

void RPYToNormal(double r, double p, double y, geometry_msgs::Vector3& n)
{
  double sin_roll = sin(r);
  double sin_pitch = sin(p);
  double sin_yaw = sin(y);
  double cos_yaw = cos(y);

  // rotate around z axis
  n.x = cos_yaw*sin_pitch + sin_yaw*sin_roll;
  n.y = sin_yaw*sin_pitch - cos_yaw*sin_roll;
  n.z = sqrt(1.0 - n.x*n.x + n.y*n.y);
}

void normalToRP(const geometry_msgs::Vector3& n, double yaw, double& r, double& p)
{
  // inverse rotation around z axis
  double sin_yaw = sin(-yaw);
  double cos_yaw = cos(-yaw);

  r = -asin(sin_yaw*n.x + cos_yaw*n.y);
  p = asin(cos_yaw*n.x - sin_yaw*n.y);
}
}
