#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <quadrotor_msgs/GoalSet.h>
#include <nav_msgs/Odometry.h>
#include <uav_utils/geometry_utils.h>

using namespace std;

// ros::Subscriber trajs_sub_;
ros::Publisher goals_pub_;
struct Drone_Info_t
{
  Eigen::Vector3d goal;
  Eigen::Vector3d last_goal;
  int goal_id{-1};
  Eigen::Vector3d cur_p;
  Eigen::Vector3d cur_v;
  double cur_yaw;
  Eigen::Vector3d last_p;
  ros::Time arrived_time;
  ros::Time set_new_goal_time;
  bool arrived_for_a_while{true};
  bool odom_received{false};
};
struct Goal_t
{
  Eigen::Vector3d p;
  bool occupied;
};
std::vector<Drone_Info_t> drones_;

void set_odom_data(const nav_msgs::Odometry::ConstPtr &msg, const int &drone_id)
{
  drones_[drone_id].odom_received = true;
  drones_[drone_id].last_p = drones_[drone_id].cur_p;
  drones_[drone_id].cur_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  drones_[drone_id].cur_v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
  drones_[drone_id].cur_yaw = uav_utils::get_yaw_from_quaternion(Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z));
}

void odoms_sim_sub_cb(const nav_msgs::Odometry::ConstPtr &msg, const int &drone_id)
{
  set_odom_data(msg, drone_id);
}

void combined_odoms_sim_sub_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  int id = atoi(msg->child_frame_id.substr(6, 10).c_str());
  if ( msg->child_frame_id.substr(0, 6) != string("drone_") || id >= (int)drones_.size())
  {
    ROS_ERROR("[random_goals_node] Wrong child_frame_id: %s, or wrong drone_id: %d", msg->child_frame_id.substr(0, 6).c_str(), id);
    return;
  }
  set_odom_data(msg, id);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_goals");
  ros::NodeHandle nh("~");

  srand(floor(ros::Time::now().toSec() * 10));

  int drone_num, goal_num;
  nh.param("drone_num", drone_num, -1);
  nh.param("goal_num", goal_num, -1);

  vector<Goal_t> goals(goal_num);
  for (int i = 0; i < goal_num; ++i)
  {
    vector<double> pt;
    nh.getParam("goal" + to_string(i), pt);
    goals[i].p << pt[0], pt[1], pt[2];
    goals[i].occupied = false;
  }

  drones_.resize(drone_num);
  std::vector<ros::Subscriber> odoms_sim_sub(drone_num);
  for (int i = 0; i < drone_num; ++i)
  {
    odoms_sim_sub[i] = nh.subscribe<nav_msgs::Odometry>("/drone_" + to_string(i) + "_visual_slam/odom", 1000, boost::bind(odoms_sim_sub_cb, _1, i));
  }

  ros::Subscriber combined_odoms_sim_sub;
  combined_odoms_sim_sub = nh.subscribe<nav_msgs::Odometry>("/others_odom", 1000, combined_odoms_sim_sub_cb);

  goals_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);

  std::vector<int> count(goal_num);
  fill(count.begin(), count.end(), 0);
  
  // Wait for 4s 
  ROS_INFO("[random goals] Wait for 5 seconds.");
  ros::Duration(5.0).sleep();

  while (ros::ok())
  {
    ros::Time t_now = ros::Time::now();

    for (int i = 0; i < drone_num; ++i)
    {
      double d_to_goal = (drones_[i].cur_p - drones_[i].goal).norm();
      double last_d_to_goal = (drones_[i].last_p - drones_[i].goal).norm();
      // cout << "last_p=" << drones_[i].last_p.transpose() << " cur_p=" << drones_[i].cur_p.transpose() << " goal=" << drones_[i].goal.transpose() << endl;
      // cout << "d_to_goal=" << d_to_goal << " last_d_to_goal=" << last_d_to_goal << endl;
      if (d_to_goal > 0.2 || last_d_to_goal > 0.2)
        drones_[i].arrived_time = t_now;
      drones_[i].arrived_for_a_while |= ((t_now - drones_[i].arrived_time).toSec() > 2);

      if ( (drones_[i].cur_p - drones_[i].last_goal).norm() < 0.3 && drones_[i].cur_v.norm() < 0.3 && (t_now - drones_[i].set_new_goal_time).toSec() > 3.0 ) // did not start
      {
        drones_[i].set_new_goal_time = t_now;
        quadrotor_msgs::GoalSet msg;
        msg.drone_id = i;
        msg.goal[0] = drones_[i].goal(0);
        msg.goal[1] = drones_[i].goal(1);
        msg.goal[2] = drones_[i].goal(2);
        goals_pub_.publish(msg);
        ROS_WARN("Drone %d doesn't start, set goal again!", i);
        cout << "drone_id=" << i << " goal=" << drones_[i].goal.transpose() << " t=" << t_now << endl;
      }
    }

    int drone_trials = 0;
    while (drone_trials < drone_num)
    {
      int d_id = floor(((double)rand() / RAND_MAX) * drone_num);
      // cout << "d_id=" << d_id << " drones_[d_id].odom_received=" << drones_[d_id].odom_received << " drones_[d_id].arrived_for_a_while=" << drones_[d_id].arrived_for_a_while << endl;
      if (drones_[d_id].odom_received && drones_[d_id].arrived_for_a_while)
      {
        int goal_trials = 0;
        while (goal_trials < goal_num)
        {
          int g_id = floor(((double)rand() / RAND_MAX) * goal_num);
          double ang = acos(((goals[g_id].p - drones_[d_id].cur_p).normalized()).dot((Eigen::Vector3d(cos(drones_[d_id].cur_yaw), sin(drones_[d_id].cur_yaw), 0)).normalized()));
          double dist_to_goal = (goals[g_id].p - drones_[d_id].cur_p).norm();
          // cout << "g_id=" << g_id << " goals[g_id].occupied=" << goals[g_id].occupied << " ang=" << ang << endl;
          if (!goals[g_id].occupied && dist_to_goal > 5.0)
          {
            if (drones_[d_id].goal_id >= 0)
              goals[drones_[d_id].goal_id].occupied = false;
            goals[g_id].occupied = true;
            drones_[d_id].arrived_for_a_while = false;
            drones_[d_id].last_goal = drones_[d_id].goal;
            drones_[d_id].goal = goals[g_id].p;
            drones_[d_id].goal_id = g_id;
            drones_[d_id].set_new_goal_time = t_now;
            quadrotor_msgs::GoalSet msg;
            msg.drone_id = d_id;
            msg.goal[0] = drones_[d_id].goal(0);
            msg.goal[1] = drones_[d_id].goal(1);
            msg.goal[2] = drones_[d_id].goal(2);
            goals_pub_.publish(msg);
            cout << "drone_id=" << d_id << " goal=" << drones_[d_id].goal.transpose() << " t=" << t_now << endl;
            goto multi_loop;
          }

          goal_trials++;
        }
      }

      drone_trials++;
    }

  multi_loop:;

    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  return 0;
}
