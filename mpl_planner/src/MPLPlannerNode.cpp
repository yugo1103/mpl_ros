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
  timer1_ = privateNodeHandle_.createTimer(ros::Duration(0.2), &MPLPlannerNode::timerCallback1, this);
  timer2_ = privateNodeHandle_.createTimer(ros::Duration(20), &MPLPlannerNode::timerCallback2, this);

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
  if (!privateNodeHandle_.param<bool>("use_yaw", useYaw_, true)) {
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
  if (!privateNodeHandle_.param<double>("goal_tolerance", goalTolerance_, 0.2)) {
    ROS_ERROR("Could not load goal_tolerance.");
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


void MPLPlannerNode::processCloud(const sensor_msgs::PointCloud& cloud) {
  Vec3f ori = startPosition_ - Vec3f(voxelRangeX_, voxelRangeY_, voxelRangeZ_) / 2;
  Vec3f dim(voxelRangeX_, voxelRangeY_, voxelRangeZ_);
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
  Vec3f ori = startPosition_ - Vec3f(voxelRangeX_, voxelRangeY_, voxelRangeZ_) / 2;
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.resolution;
  std::vector<signed char> map = msg.data;

  mapUtilPtr_->setMap(ori, dim, map, res);
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
  mapUtilPtr_->freeUnknown();
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

void MPLPlannerNode::planTrajectory()
{
  if(plannerPtr_->goal_pose_change_){
    plannerPtr_->replanning_flag_ = true;
  }

  if(!plannerPtr_->replanning_flag_){
    if(plannerPtr_->check_traj()){
      return;
    }
  }
  
  double diff = (goalPosition_ - startPosition_).norm();
  if(diff < goalTolerance_) 
  {
    trajectory_msgs::MultiDOFJointTrajectory commandTrajectory;
    commandTrajectory.header.frame_id = "world";
    trajectory_msgs::MultiDOFJointTrajectoryPoint tp;
    geometry_msgs::Transform transform;
    transform.translation.x = goalPosition_.x();
    transform.translation.y = goalPosition_.y();
    transform.translation.z = goalPosition_.z();
      tp.transforms.push_back(transform);
      commandTrajectory.points.push_back(tp);
      commandTrajectoryPublisher_.publish(commandTrajectory);
      return;
  }else{
    trajectory_msgs::MultiDOFJointTrajectory commandTrajectory;
    commandTrajectory.header.frame_id = "world";
    trajectory_msgs::MultiDOFJointTrajectoryPoint tp;
    geometry_msgs::Transform transform;
    transform.translation.x = startPosition_.x();
    transform.translation.y = startPosition_.y();
    transform.translation.z = startPosition_.z();
      tp.transforms.push_back(transform);
      commandTrajectory.points.push_back(tp);
      commandTrajectoryPublisher_.publish(commandTrajectory);
  }

  // Vec3f is Vector3d
  Waypoint3D start;
  start.pos = startPosition_;
  start.vel = startVelocity_;
  start.acc = Vec3f(0, 0, 0);
  start.jrk = Vec3f(0, 0, 0);
  start.yaw = currentYaw_;
  start.use_pos = true;
  start.use_vel = true;
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
    plannerPtr_->replanning_flag_ = true;
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(),
             plannerPtr_->getCloseSet().size());
  } else {
    plannerPtr_->replanning_flag_ = false;
    plannerPtr_->goal_pose_change_ = false;
    ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(),
             plannerPtr_->getCloseSet().size());

    auto traj = plannerPtr_->getTraj();

    // Publish trajectory
    std_msgs::Header header;
    header.frame_id = std::string("map");
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    // traj_msg.header = header;
    // traj_pub.publish(traj_msg);

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
    traj = traj_solver.solve();

    // Publish refined trajectory
    planning_ros_msgs::Trajectory refined_traj_msg = toTrajectoryROSMsg(traj);
    refined_traj_msg.header.frame_id = "world";
    refinedTrajectoryPublisher.publish(refined_traj_msg);

    printf("Refined traj -- J(VEL): %f, J(ACC): %f, J(JRK): %f, J(SNP): %f, J(YAW): %f, total time: %f\n",
           traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::JRK),
           traj.J(Control::SNP), traj.Jyaw(), traj.getTotalTime());
    auto refined_points = traj.sample(numberOfPoints_);
    trajectory_msgs::MultiDOFJointTrajectory commandTrajectory;
    commandTrajectory = toMultiDOFJointTrajectoryMsg(traj, numberOfPoints_);
    commandTrajectory.header.frame_id = "world";
    commandTrajectoryPublisher_.publish(commandTrajectory);
  }

  // Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(plannerPtr_->getCloseSet());
  //sensor_msgs::PointCloud ps = vec_to_cloud(planner_ptr->getValidRegion());
  // ps.header = header;
  // cloud_pub.publish(ps);
}

void MPLPlannerNode::goalPoseCallback(const geometry_msgs::PoseStamped& message)
{
  if(goalPosition_.x() != message.pose.position.x || goalPosition_.y() != message.pose.position.y || goalPosition_.z() != message.pose.position.z){
    plannerPtr_->goal_pose_change_ = true;
  }
  goalPosition_.x() = message.pose.position.x;
  goalPosition_.y() = message.pose.position.y;
  goalPosition_.z() = message.pose.position.z;
  Eigen::Quaterniond q;
  q.x() = message.pose.orientation.x;
  q.y() = message.pose.orientation.y;
  q.z() = message.pose.orientation.z;
  q.w() = message.pose.orientation.w;
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  goalYaw_ = euler[2];
  if ((goalPosition_ - startPosition_).norm() < 0.01) {
    if (fabs(goalYaw_ - currentYaw_) > 0.1) {
      commandPosePublisher_.publish(message);
    }
  }
}

void MPLPlannerNode::timerCallback1(const ros::TimerEvent&){
  planTrajectory();
}

void MPLPlannerNode::timerCallback2(const ros::TimerEvent&){
  plannerPtr_->replanning_flag_ = true;
}


//
} /* namespace */
