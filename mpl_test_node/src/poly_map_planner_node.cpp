#include <decomp_ros_utils/data_ros_utils.h>
#include <mpl_external_planner/poly_map_planner/poly_map_planner.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <ros/ros.h>

using namespace MPL;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher poly_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons", 1, true);
  ros::Publisher bound_pub =
    nh.advertise<decomp_ros_msgs::PolyhedronArray>("bound", 1, true);
  ros::Publisher sg_pub =
      nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  ros::Publisher traj_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  ros::Publisher ps_pub = nh.advertise<sensor_msgs::PointCloud>("ps", 1, true);

  vec_E<PolyhedronObstacle2D> static_obs;
  vec_E<PolyhedronLinearObstacle2D> linear_obs;

  Polyhedron2D rec1;
  rec1.add(Hyperplane2D(Vec2f(5.5, 2.5), -Vec2f::UnitX()));
  rec1.add(Hyperplane2D(Vec2f(6.5, 2.5), Vec2f::UnitX()));
  rec1.add(Hyperplane2D(Vec2f(6, 1.0), -Vec2f::UnitY()));
  rec1.add(Hyperplane2D(Vec2f(6, 4.0), Vec2f::UnitY()));
  linear_obs.push_back(PolyhedronLinearObstacle2D(rec1, Vec2f::Zero(), Vec2f(-0.5, 0)));

  Polyhedron2D rec2;
  rec2.add(Hyperplane2D(Vec2f(15.5, 4.5), -Vec2f::UnitX()));
  rec2.add(Hyperplane2D(Vec2f(16.5, 4.5), Vec2f::UnitX()));
  rec2.add(Hyperplane2D(Vec2f(16, 3.0), -Vec2f::UnitY()));
  rec2.add(Hyperplane2D(Vec2f(16, 6.0), Vec2f::UnitY()));
  static_obs.push_back(PolyhedronObstacle2D(rec2, Vec2f::Zero()));

  Polyhedron2D rec3;
  rec3.add(Hyperplane2D(Vec2f(9.5, 0.5), -Vec2f::UnitX()));
  rec3.add(Hyperplane2D(Vec2f(16.5, 0.5), Vec2f::UnitX()));
  rec3.add(Hyperplane2D(Vec2f(14, 0.0), -Vec2f::UnitY()));
  rec3.add(Hyperplane2D(Vec2f(14, 1.0), Vec2f::UnitY()));
  linear_obs.push_back(PolyhedronLinearObstacle2D(rec3, Vec2f::Zero(), Vec2f(0.0, 0.25)));

  Polyhedron2D rec4;
  rec4.add(Hyperplane2D(Vec2f(15.5, 3.0), -Vec2f::UnitX()));
  rec4.add(Hyperplane2D(Vec2f(16.5, 3.0), Vec2f::UnitX()));
  rec4.add(Hyperplane2D(Vec2f(16, 2.0), -Vec2f::UnitY()));
  rec4.add(Hyperplane2D(Vec2f(16, 3.0), Vec2f::UnitY()));
  linear_obs.push_back(PolyhedronLinearObstacle2D(rec4, Vec2f::Zero(), Vec2f(0.0, -0.1)));

  Polyhedron2D rec5;
  rec5.add(Hyperplane2D(Vec2f(6.5, 0.5), -Vec2f::UnitX()));
  rec5.add(Hyperplane2D(Vec2f(15.5, 0.5), Vec2f::UnitX()));
  rec5.add(Hyperplane2D(Vec2f(7, -2.0), -Vec2f::UnitY()));
  rec5.add(Hyperplane2D(Vec2f(7, -1.0), Vec2f::UnitY()));
  linear_obs.push_back(PolyhedronLinearObstacle2D(rec5, Vec2f::Zero(), Vec2f(-0.1, 0.3)));


  Vec2f origin, dim;
  nh.param("origin_x", origin(0), 0.0);
  nh.param("origin_y", origin(1), 0.0);
  nh.param("range_x", dim(0), 20.0);
  nh.param("range_y", dim(1), 5.0);

  // Initialize planner
  double dt, v_max, a_max, w, epsilon, t_max;
  double u;
  int num;
  nh.param("dt", dt, 1.0);
  nh.param("epsilon", epsilon, 1.0);
  nh.param("v_max", v_max, -1.0);
  nh.param("a_max", a_max, -1.0);
  nh.param("u", u, 1.0);
  nh.param("w", w, 10.);
  nh.param("num", num, 1);

  std::unique_ptr<MPL::PolyMapPlanner2D> planner_ptr;
  planner_ptr.reset(new MPL::PolyMapPlanner2D(true));
  planner_ptr->setMap(origin, dim);         // Set collision checking function
  planner_ptr->setStaticObstacles(static_obs);
  planner_ptr->setLinearObstacles(linear_obs);
  planner_ptr->setEpsilon(epsilon); // Set greedy param (default equal to 1)
  planner_ptr->setVmax(v_max);      // Set max velocity
  planner_ptr->setAmax(a_max);      // Set max acceleration
  planner_ptr->setDt(dt);           // Set dt for each primitive
  planner_ptr->setW(w);             // Set w for each primitive
  planner_ptr->setTol(0.5); // Tolerance for goal region

  // Set start and goal
  double start_x, start_y;
  nh.param("start_x", start_x, 0.5);
  nh.param("start_y", start_y, 1.0);
  double start_vx, start_vy;
  nh.param("start_vx", start_vx, 0.0);
  nh.param("start_vy", start_vy, 0.0);
  double goal_x, goal_y;
  nh.param("goal_x", goal_x, 6.4);
  nh.param("goal_y", goal_y, 16.6);

  bool use_acc, use_jrk;
  nh.param("use_acc", use_acc, false);
  nh.param("use_jrk", use_jrk, false);

  Waypoint2D start;
  start.pos = Vec2f(start_x, start_y);
  start.vel = Vec2f(start_vx, start_vy);
  start.acc = Vec2f::Zero();
  start.jrk = Vec2f::Zero();
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = use_acc;
  start.use_jrk = use_jrk;
  start.use_yaw = false;

  Waypoint2D goal(start.control);
  goal.pos = Vec2f(goal_x, goal_y);
  goal.vel = Vec2f::Zero();
  goal.acc = Vec2f::Zero();
  goal.jrk = Vec2f::Zero();

  // Set input control
  vec_E<VecDf> U;
  const decimal_t du = u / num;
  for (decimal_t dx = -u; dx <= u; dx += du)
    for (decimal_t dy = -u; dy <= u; dy += du)
      U.push_back(Vec2f(dx, dy));
  planner_ptr->setU(U); // Set discretization with 1 and efforts

  auto t0 = ros::Time::now();
  if(!planner_ptr->plan(start, goal)) {
    ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
             (ros::Time::now() - t0).toSec(), planner_ptr->getCloseSet().size());
    return -1;
  }
  ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
           (ros::Time::now() - t0).toSec(), planner_ptr->getCloseSet().size());

  // Publish trajectory
  auto traj = planner_ptr->getTraj();
  planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
  traj_msg.header.frame_id = "map";
  traj_pub.publish(traj_msg);

  printf("================== Traj -- total J(VEL): %f, J(ACC): %F, J(JRK): %f, "
         "total time: %f\n",
         traj.J(Control::VEL), traj.J(Control::ACC), traj.J(Control::SNP), traj.getTotalTime());

  // Publish expanded nodes
  sensor_msgs::PointCloud ps = vec_to_cloud(vec2_to_vec3(planner_ptr->getCloseSet()));
  ps.header.frame_id = "map";
  ps_pub.publish(ps);

  vec_E<Polyhedron2D> bbox;
  bbox.push_back(planner_ptr->getBoundingBox());
  decomp_ros_msgs::PolyhedronArray bbox_msg = DecompROS::polyhedron_array_to_ros(bbox);
  bbox_msg.header.frame_id = "map";
  bound_pub.publish(bbox_msg);

  ros::Rate loop_rate(10);

  t0 = ros::Time::now();
  while (ros::ok()) {
    double dt = (ros::Time::now() - t0).toSec();
    if(dt < traj.getTotalTime()) {
      vec_E<Polyhedron2D> poly_obs = planner_ptr->getPolyhedrons(dt);
      decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly_obs);
      poly_msg.header.frame_id = "map";
      poly_pub.publish(poly_msg);

      auto curr = traj.evaluate(dt);
      // Publish location of start and goal
      sensor_msgs::PointCloud sg_cloud;
      sg_cloud.header.frame_id = "map";
      geometry_msgs::Point32 pt1, pt2;
      pt1.x = curr.pos(0), pt1.y = curr.pos(1), pt1.z = 0;
      pt2.x = goal_x, pt2.y = goal_y, pt2.z = 0;
      sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
      sg_pub.publish(sg_cloud);
    }
    else
      t0 = ros::Time::now();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
