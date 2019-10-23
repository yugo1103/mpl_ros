/*
 * MPLPlannerNode.cpp
 *
 *  Created on: Jul 23, 2018
 *      Author: Takahiro Miki
 *	 Institute: Univ of Tokyo AILab, HongoAerospace.inc
 */

#include "mpl_planner/MPLPlannerNode.hpp"

#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>
#include <mpl_traj_solver/traj_solver.h>
#include <planning_ros_msgs/Trajectory.h>
#include "std_msgs/String.h"


#include <random>

using namespace MPL;

namespace mpl_planner {

MPLPlannerNode::MPLPlannerNode(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle)
    : nodeHandle_(nodeHandle),
      privateNodeHandle_(privateNodeHandle),
      mapUtilPtr_(new MPL::VoxelMapUtil),
      voxelGridPtr_(new VoxelGrid(Vec3f(0, 0, 0), Vec3f(5, 5, 5), 0.2)),
      plannerPtr_(new MPL::VoxelMapPlanner(false))
{
  readParameters();
  goalPosition_ = waypoints_[0];
  setupMPLPlanner();
  ROS_INFO("Finished mpl setup");
  trajectoryPublisher_ = privateNodeHandle_.advertise<visualization_msgs::Marker>("line_marker", 0);
  refinedTrajectoryPublisher = privateNodeHandle_.advertise<planning_ros_msgs::Trajectory>(
      "trajectory_refined", 1, true);

  commandTrajectoryPublisher_ = nodeHandle_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                                                                  "/firefly/command/trajectory", 0);
  commandPosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 0);
  voxelMapPublisher_ =
      privateNodeHandle_.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  odometrySubscriber_ = privateNodeHandle_.subscribe("odometry", 1,
                                        &MPLPlannerNode::odometryCallback, this);
  goalPoseSubscriber_ = privateNodeHandle_.subscribe("goal_pose", 1,
                                        &MPLPlannerNode::goalPoseCallback, this);
  cloudSubscriber_ = privateNodeHandle_.subscribe("cloud", 1,
                                        &MPLPlannerNode::cloudCallback, this);


  planTrajectory();
  timer1_ = privateNodeHandle_.createTimer(ros::Duration(0.2), &MPLPlannerNode::timerCallback1, this);
  timer2_ = privateNodeHandle_.createTimer(ros::Duration(replanningSpan_), &MPLPlannerNode::timerCallback2, this);

}

MPLPlannerNode::~MPLPlannerNode()
{
}

bool MPLPlannerNode::readParameters()
{
  if (!privateNodeHandle_.param<bool>("planner_verbose", plannerVerbose_, true)) {
    ROS_ERROR("Could not load planner_verbose.");
  }
  if (!privateNodeHandle_.param<int>("number_of_points", numberOfPoints_, 100)) {
    ROS_ERROR("Could not load number of points.");
  }
  if (!privateNodeHandle_.param<double>("robot_radius", robotRadius_, 0.3)) {
    ROS_ERROR("Could not load robot radius.");
  }
  if (!privateNodeHandle_.param<double>("voxel_range_x", voxelRangeX_, 5.0)) {
    ROS_ERROR("Could not load voxel range x.");
  }
  if (!privateNodeHandle_.param<double>("voxel_range_y", voxelRangeY_, 5.0)) {
    ROS_ERROR("Could not load voxel range y.");
  }
  if (!privateNodeHandle_.param<double>("voxel_range_z", voxelRangeZ_, 5.0)) {
    ROS_ERROR("Could not load voxel range z.");
  }
  if (!privateNodeHandle_.param<double>("voxel_resolution", voxelResolution_, 0.2)) {
    ROS_ERROR("Could not load voxel resolution.");
  }
  if (!privateNodeHandle_.param<bool>("use_3d", use3d_, true)) {
    ROS_ERROR("Could not load use_3d.");
  }
  if (!privateNodeHandle_.param<bool>("use_yaw", useYaw_, false)) {
    ROS_ERROR("Could not load use_yaw.");
  }
  if (!privateNodeHandle_.param<double>("vel_max", velMax_, 2.0)) {
    ROS_ERROR("Could not load vel_max.");
  }
  if (!privateNodeHandle_.param<double>("acc_max", accMax_, 1.0)) {
    ROS_ERROR("Could not load acc_max.");
  }
  if (!privateNodeHandle_.param<double>("jrk_max", jrkMax_, 1.0)) {
    ROS_ERROR("Could not load jrk_max.");
  }
  if (!privateNodeHandle_.param<double>("yaw_max", yawMax_, 1.0)) {
    ROS_ERROR("Could not load yaw_max.");
  }
  if (!privateNodeHandle_.param<double>("primitive_u", primitiveU_, 1.0)) {
    ROS_ERROR("Could not load primitive_u.");
  }
  if (!privateNodeHandle_.param<double>("primitive_u_yaw", primitiveUYaw_, 0.3)) {
    ROS_ERROR("Could not load primitive_u_yaw.");
  }
  if (!privateNodeHandle_.param<int>("primitive_u_num", primitiveUNum_, 1)) {
    ROS_ERROR("Could not load primitive_u_num.");
  }
  if (!privateNodeHandle_.param<double>("dt", dt_, 0.2)) {
    ROS_ERROR("Could not load dt.");
  }
  if (!privateNodeHandle_.param<int>("ndt", ndt_, 10)) {
    ROS_ERROR("Could not load ndt.");
  }
  if (!privateNodeHandle_.param<double>("replanning_span", replanningSpan_, 20.0)) {
    ROS_ERROR("Could not load replanning_span.");
  }
  if (!privateNodeHandle_.param<double>("goal_tolerance", goalTolerance_, 0.2)) {
    ROS_ERROR("Could not load goal_tolerance.");
  }

  Eigen::Vector3d wp;

  if (!privateNodeHandle_.param<double>("waypoints_x_" + std::to_string(1), wp(0), 0)) {
    ROS_ERROR("Could not load waypoints_x.");
  }
  if (!privateNodeHandle_.param<double>("waypoints_y_" + std::to_string(1), wp(1), 0)) {
    ROS_ERROR("Could not load waypoints_y.");
  }
  if (!privateNodeHandle_.param<double>("waypoints_z_" + std::to_string(1), wp(2), 0)) {
    ROS_ERROR("Could not load waypoints_z.");
  }

  waypoints_.push_back(wp);

  if (!privateNodeHandle_.param<int>("waypoints_num", waypointsNum_, 0)) {
    ROS_ERROR("Could not load waypoints_num");
  }

  for(int i = 1; i < waypointsNum_; ++i)
  {

    if (!privateNodeHandle_.param<double>("waypoints_x_" + std::to_string(i + 1), wp(0), 0)) {
    ROS_ERROR("Could not load waypoints_x_%d.",i);
  }
  if (!privateNodeHandle_.param<double>("waypoints_y_" + std::to_string(i + 1), wp(1), 0)) {
    ROS_ERROR("Could not load waypoints_y_%d.",i);
  }
  if (!privateNodeHandle_.param<double>("waypoints_z_" + std::to_string(i + 1), wp(2), 0)) {
    ROS_ERROR("Could not load waypoints_z_%d.",i);
  }

    waypoints_.push_back(wp);
  }


}

void MPLPlannerNode::setupMPLPlanner() {
  vec_E<VecDf> U; // Control input
  const decimal_t du = primitiveU_ / primitiveUNum_;
  const decimal_t u = primitiveU_;
  const decimal_t u_yaw = primitiveUYaw_;
  if (use3d_ && !useYaw_) {
    // consider primitive in z-axis, without yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        for (decimal_t dz = -u; dz <= u; dz += du)
          U.push_back(Vec3f(dx, dy, dz));
  }
  else if(!use3d_ && !useYaw_) {
    // consider 2D primitive, without yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        U.push_back(Vec3f(dx, dy, 0));
  }
  else if(!use3d_ && useYaw_) {
    // consider 2D primitive, with yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        for (decimal_t dyaw = -u_yaw; dyaw <= u_yaw; dyaw += u_yaw) {
          Vec4f vec;
          vec << dx, dy, 0, dyaw;
          U.push_back(vec);
        }
  }
  else if(use3d_ && useYaw_) {
    // consider primitive in z-axis, with yawing
    for (decimal_t dx = -u; dx <= u; dx += du)
      for (decimal_t dy = -u; dy <= u; dy += du)
        for (decimal_t dz = -u; dz <= u; dz += du)
          for (decimal_t dyaw = -u_yaw; dyaw <= u_yaw; dyaw += u_yaw) {
            Vec4f vec;
            vec << dx, dy, dz, dyaw;
            U.push_back(vec);
          }
  }
  // controlInput_ = U;
  mapUtilPtr_.reset(new MPL::VoxelMapUtil);
  plannerPtr_.reset(new MPL::VoxelMapPlanner(plannerVerbose_));
  plannerPtr_->setMapUtil(mapUtilPtr_); // Set collision checking function
  plannerPtr_->setVmax(velMax_);       // Set max velocity
  plannerPtr_->setAmax(accMax_);       // Set max acceleration (as control input)
  plannerPtr_->setJmax(jrkMax_);       // Set max acceleration (as control input)
  plannerPtr_->setYawmax(yawMax_);       // Set yaw threshold
  plannerPtr_->setDt(dt_);            // Set dt for each primitive
  plannerPtr_->setTmax(ndt_ * dt_);    // Set the planning horizon: n*dt
  plannerPtr_->setU(U); // Set control input
  plannerPtr_->setTol(goalTolerance_); // Tolerance for goal region

  Vec3f origin, dim;
  dim(0) = voxelRangeX_;
  dim(1) = voxelRangeY_;
  dim(2) = voxelRangeZ_;
  origin = -dim / 2;
  // origin(0) = 0;
  // origin(1) = 0;
  origin(2) = 0;

  voxelGridPtr_.reset(new VoxelGrid(origin, dim, voxelResolution_));
}

//PointCloudからマップを作成しpublish
void MPLPlannerNode::processCloud(const sensor_msgs::PointCloud& cloud) {
  //マップ原点
  Vec3f ori = startPosition_ - Vec3f(voxelRangeX_, voxelRangeY_, voxelRangeZ_) / 2;
  //マップ範囲
  // Free unknown space and dilate obstacles
  Vec3f dim(voxelRangeX_, voxelRangeY_, voxelRangeZ_);
  voxelGridPtr_->clear();
  voxelGridPtr_->allocate(dim, ori);
  voxelGridPtr_->addCloud(cloud_to_vec(cloud));
  planning_ros_msgs::VoxelMap map = voxelGridPtr_->getMap();
  setVoxelMap(map);
  map.header = cloud.header;
  voxelMapPublisher_.publish(map);
  // ROS_INFO("Processed the voxel map! [%zu]", cloud.points.size());
}

void MPLPlannerNode::cloudCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
  if(msg->getDataType() == "sensor_msgs/PointCloud") {
    auto cloud_ptr = msg->instantiate<sensor_msgs::PointCloud>();
    processCloud(*cloud_ptr);
  }
  //入力がPointCloud2のときPointCloudへ変換
  else if(msg->getDataType() == "sensor_msgs/PointCloud2") {
    auto cloud2_ptr = msg->instantiate<sensor_msgs::PointCloud2>();
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud2_ptr, cloud);
    processCloud(cloud);
  }
  else
    return;
}

void MPLPlannerNode::setMap(const planning_ros_msgs::VoxelMap& msg) {
  // Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  //マップ原点
  Vec3f ori = startPosition_ - Vec3f(voxelRangeX_, voxelRangeY_, voxelRangeZ_) / 2;
  //マップ範囲
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.resolution;
  std::vector<signed char> map = msg.data;

  mapUtilPtr_->setMap(ori, dim, map, res);
}


void MPLPlannerNode::expandVoxelResolution(const double expand_size) {
  planning_ros_msgs::VoxelMap map = voxelGridPtr_->getMap();
  setMap(map);
  // Inflate obstacle using robot radius (>0)
  if(expand_size > 0) {
    vec_Vec3i ns;
    int rn = std::ceil(expand_size / mapUtilPtr_->getRes());
    for(int nx = -rn; nx <= rn; nx++) {
      for(int ny = -rn; ny <= rn; ny++) {
        if(nx == 0 && ny == 0)
          continue;
        if(std::hypot(nx, ny) > rn)
          continue;
        ns.push_back(Vec3i(nx, ny, 0));
      }
    }
    mapUtilPtr_->dilate(ns);
    mapUtilPtr_->freeRobot_r(expand_size);
  }

  getMap(map);
  map.header.frame_id = "world";
  voxelMapPublisher_.publish(map);
}

void MPLPlannerNode::getMap(planning_ros_msgs::VoxelMap &map) {
  Vec3f ori = mapUtilPtr_->getOrigin();
  Vec3i dim = mapUtilPtr_->getDim();
  decimal_t res = mapUtilPtr_->getRes();

  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.resolution = res;

  map.data = mapUtilPtr_->getMap();
}

void MPLPlannerNode::setVoxelMap(planning_ros_msgs::VoxelMap& map) {
  setMap(map);
  // Free unknown space and dilate obstacles
  //mapUtilPtr_->freeUnknown();
  // Inflate obstacle using robot radius (>0)
  if(robotRadius_ > 0) {
    vec_Vec3i ns;
    int rn = std::ceil(robotRadius_ / mapUtilPtr_->getRes());
    for(int nx = -rn; nx <= rn; nx++) {
      for(int ny = -rn; ny <= rn; ny++) {
        if(nx == 0 && ny == 0)
          continue;
        if(std::hypot(nx, ny) > rn)
          continue;
        ns.push_back(Vec3i(nx, ny, 0));
      }
    }
    mapUtilPtr_->dilate(ns);
    mapUtilPtr_->freeRobot_r(robotRadius_);
  }


  // Publish the dilated map for visualization
  getMap(map);
  // voxelMapPublisher_.publish(map);

}


void MPLPlannerNode::odometryCallback(const nav_msgs::Odometry& msg) {
  startPosition_.x() = msg.pose.pose.position.x;
  startPosition_.y() = msg.pose.pose.position.y;
  startPosition_.z() = msg.pose.pose.position.z;
  startVelocity_.x() = msg.twist.twist.linear.x;
  startVelocity_.y() = msg.twist.twist.linear.y;
  startVelocity_.z() = msg.twist.twist.linear.z;
  Eigen::Quaterniond q;
  q.x() = msg.pose.pose.orientation.x;
  q.y() = msg.pose.pose.orientation.y;
  q.z() = msg.pose.pose.orientation.z;
  q.w() = msg.pose.pose.orientation.w;
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  currentYaw_ = euler[2];
  // ROS_INFO_STREAM("got odometry" << msg);
}

void MPLPlannerNode::publishTrajectry(const Trajectory3D& traj) {
    trajectory_msgs::MultiDOFJointTrajectory commandTrajectory;
    commandTrajectory = toMultiDOFJointTrajectoryMsg(traj, numberOfPoints_);
    commandTrajectory.header.frame_id = "world";

    //yaw calculation
    for(int i = 0; i < numberOfPoints_ - 1; i++){
      geometry_msgs::Quaternion quat_Msg;
      quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, atan2((commandTrajectory.points[i + 1].transforms[0].translation.y - commandTrajectory.points[i].transforms[0].translation.y), (commandTrajectory.points[i + 1].transforms[0].translation.x - commandTrajectory.points[i].transforms[0].translation.x))), quat_Msg);
      commandTrajectory.points[i].transforms[0].rotation = quat_Msg;
    }


    commandTrajectoryPublisher_.publish(commandTrajectory);
    publishTrajectryLine(traj);
}

void MPLPlannerNode::publishTrajectry(const Eigen::Vector3d& goal_point) {
	trajectory_msgs::MultiDOFJointTrajectory commandTrajectory;
	commandTrajectory.header.frame_id = "world";
	trajectory_msgs::MultiDOFJointTrajectoryPoint tp;
	geometry_msgs::Transform transform;
	transform.translation.x = goal_point.x();
	transform.translation.y = goal_point.y();
	transform.translation.z = goal_point.z();
	tp.transforms.push_back(transform);
	commandTrajectory.points.push_back(tp);
	commandTrajectoryPublisher_.publish(commandTrajectory);
}

void MPLPlannerNode::publishTrajectryLine(const Trajectory3D& traj) {
    //経路可視化(rviz用)
    std_msgs::Header header;
    header.frame_id = std::string("map");
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);

    printf("Raw traj -- J(VEL): %f, J(ACC): %f, J(JRK): %f, J(SNP): %f, J(YAW): %f, total time: %f\n",
           traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::JRK),
           traj.J(Control::SNP), traj.Jyaw(), traj.getTotalTime());

    // Get intermediate waypoints
    auto waypoints = traj.getWaypoints();
    for(int i = 1; i < waypoints.size() - 1; i++)
      waypoints[i].control = Control::VEL;
    // Get time allocation
    auto dts = traj.getSegmentTimes();

    // Generate higher order polynomials
    // TrajSolver3D traj_solver(Control::JRK);
    TrajSolver3D traj_solver(Control::ACC);
    traj_solver.setWaypoints(waypoints);
    traj_solver.setDts(dts);

    Trajectory3D refined_traj;
    refined_traj = traj_solver.solve();

    // Publish refined trajectory
    planning_ros_msgs::Trajectory refined_traj_msg = toTrajectoryROSMsg(refined_traj);
    refined_traj_msg.header.frame_id = "world";
    refinedTrajectoryPublisher.publish(refined_traj_msg);

    printf("Refined traj -- J(VEL): %f, J(ACC): %f, J(JRK): %f, J(SNP): %f, J(YAW): %f, total time: %f\n",
           refined_traj.J(Control::VEL), refined_traj.J(Control::ACC), refined_traj.J(Control::JRK),
           refined_traj.J(Control::SNP), refined_traj.Jyaw(), refined_traj.getTotalTime());
}

void MPLPlannerNode::planTrajectory()
{
	double diff = (goalPosition_ - startPosition_).norm();
    //現在地からgoalが十分近い時,経路生成せず直接移動
  	if(diff < goalTolerance_ * 2)
  	{
        publishTrajectry(goalPosition_);
	    return;
	}

    //経路計算の間その場に留まる
    publishTrajectry(startPosition_);

    //障害物を大きく回避するためvoxel拡大
    expandVoxelResolution(robotRadius_ * 1.5);

  // Vec3f is Vector3d
  Waypoint3D start;
  start.pos = startPosition_;
  start.vel = startVelocity_;
  start.acc = Vec3f(0, 0, 0);
  start.jrk = Vec3f(0, 0, 0);
  start.yaw = currentYaw_;
  start.use_pos = true;
  start.use_vel = false;
  start.use_acc = false;
  start.use_jrk = false;
  start.use_yaw = useYaw_; // if true, yaw is also propogated

  Waypoint3D goal(start.control); // initialized with the same control as start
  goal.pos = goalPosition_;
  goal.vel = Vec3f(0, 0, 0);
  goal.acc = Vec3f(0, 0, 0);
  goal.jrk = Vec3f(0, 0, 0);

  // Planning thread!
  ros::Time t0 = ros::Time::now();
  bool valid = plannerPtr_->plan(start, goal);

  if (!valid) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(),
             plannerPtr_->getCloseSet().size());
    is_goal_ = false;
    return;
  } else {
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(),
             plannerPtr_->getCloseSet().size());

    auto traj = plannerPtr_->getTraj();
    publishTrajectry(traj);
    is_goal_ = true;
  }

}

void MPLPlannerNode::goalPoseCallback(const geometry_msgs::PoseStamped& message)
{
  Eigen::Quaterniond q;
  q.x() = message.pose.orientation.x;
  q.y() = message.pose.orientation.y;
  q.z() = message.pose.orientation.z;
  q.w() = message.pose.orientation.w;
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  goalYaw_ = euler[2];
  //ゴール座標が変わった時経路再計算
  if(goalPosition_.x() != message.pose.position.x || goalPosition_.y() != message.pose.position.y || goalPosition_.z() != message.pose.position.z){
      goalPosition_.x() = message.pose.position.x;
      goalPosition_.y() = message.pose.position.y;
      goalPosition_.z() = message.pose.position.z;
      planTrajectory();
  }
}

//経路上に障害物がないか定期的に確認
//処理に時間がかからないようならcloudCallbackに移行
void MPLPlannerNode::timerCallback1(const ros::TimerEvent&){
  if(!plannerPtr_->check_traj() || !is_goal_){
      planTrajectory();
  }
}

//定期的に経路再計算
void MPLPlannerNode::timerCallback2(const ros::TimerEvent&){
    planTrajectory();
}


//
} /* namespace */
