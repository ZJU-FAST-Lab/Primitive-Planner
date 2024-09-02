#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

using std::vector;

namespace primitive_planner
{

  struct LocalTrajData
  {
    int drone_id;
    double start_time;
    double traj_duration;
    std::vector<Eigen::Vector3d> traj_pos;
  };

  typedef std::vector<LocalTrajData> SwarmTrajData;


} // namespace primitive_planner

#endif