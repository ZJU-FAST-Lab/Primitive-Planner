#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/pp_replan_fsm.h>

using namespace primitive_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "primitive_planner_node");
  ros::NodeHandle nh("~");

  PPReplanFSM rebo_replan;

  rebo_replan.init(nh);

  ros::spin();

  return 0;
}
