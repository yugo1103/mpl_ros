/*
 * mpl_planner_node.cpp
 *
 *  Created on: Jul 24, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab
 */

#include <ros/ros.h>
#include <mpl_planner/MPLPlannerNode.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpl_planner");

  ros::NodeHandle nodeHandle;
  ros::NodeHandle privateNodeHandle("~");
  mpl_planner::MPLPlannerNode mpl_planner_node(nodeHandle, privateNodeHandle);
  // ros::Rate rate(1.0);
  // while (nodeHandle.ok()) {
  //   trajectoryPlannerRos.publishSubMap(grid_map::Position(0, 0), grid_map::Length(1, 1));
  //   rate.sleep();
  // }


  ros::spin();
  return 0;
}
