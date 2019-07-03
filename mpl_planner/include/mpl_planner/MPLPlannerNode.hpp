/*
 * MPLPlannerNode.hpp
 *
 *  Created on: Jul 23, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 *
 */

#pragma once

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <mpl_planner/planner/map_planner.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <planning_ros_utils/voxel_grid.h>
#include <topic_tools/shape_shifter.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <vector>

using namespace MPL;

namespace mpl_planner {

class MPLPlannerNode
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  MPLPlannerNode(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle);

  /*!
   * Destructor.
   */
  virtual ~MPLPlannerNode();

 private:
  bool readParameters();
  void setupMPLPlanner();
  void setMap(const planning_ros_msgs::VoxelMap& msg);
  void getMap(planning_ros_msgs::VoxelMap &map);
  void setVoxelMap(planning_ros_msgs::VoxelMap& map);
  void processCloud(const sensor_msgs::PointCloud& cloud);

  // void publishCommandTrajectory(const std::vector<Eigen::Vector4d>& trajectory);
  // void publishLine(const std::vector<Eigen::Vector4d>& trajectory);
  void goalPoseCallback(const geometry_msgs::PoseStamped& message);
  void odometryCallback(const nav_msgs::Odometry& message);
  void cloudCallback(const topic_tools::ShapeShifter::ConstPtr &msg);

  // void esdfMapCallback(const voxblox_msgs::LayerConstPtr layerMsg);
  void planTrajectory();

  void timerCallback1(const ros::TimerEvent&);
  void timerCallback2(const ros::TimerEvent&);

  // Planning variables
  std::shared_ptr<MPL::VoxelMapUtil> mapUtilPtr_;
  std::unique_ptr<MPL::VoxelMapPlanner> plannerPtr_;
  std::unique_ptr<VoxelGrid> voxelGridPtr_;
  vec_E<VecDf> controlInput_;
  Eigen::Vector3d startPosition_;
  Eigen::Vector3d startVelocity_;
  Eigen::Vector3d goalPosition_;
  std::vector<Eigen::Vector3d> waypoints_;
  int waypoints_counter_ = 0;
  int waypoints_num_;

  double currentYaw_;
  double goalYaw_;

  // Coordinate parameters
  std::string baseFrameId_;
  bool plannerVerbose_;
  double robotRadius_;
  double voxelRangeX_;
  double voxelRangeY_;
  double voxelRangeZ_;
  double voxelResolution_;
  bool use3d_;
  bool useYaw_;
  double velMax_;
  double accMax_;
  double jrkMax_;
  double yawMax_;
  double primitiveU_;
  double primitiveUYaw_;
  int primitiveUNum_;
  double dt_;
  int ndt_;
  double goalTolerance_;
  double replanning_span_;
  int numberOfPoints_;

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle privateNodeHandle_;

  //! Goal/commamd topics.
  ros::Publisher commandTrajectoryPublisher_;
  ros::Publisher commandPosePublisher_;
  ros::Subscriber goalPoseSubscriber_;
  ros::Subscriber startPoseSubscriber_;
  ros::Subscriber odometrySubscriber_;
  ros::Subscriber cloudSubscriber_;

  // ROS publisher
  ros::Publisher trajectoryPublisher_;
  ros::Publisher voxelMapPublisher_;
  ros::Publisher refinedTrajectoryPublisher;
  ros::Publisher markerPublisher_;

  ros::Timer timer1_;
  ros::Timer timer2_;

};

} /* namespace */
