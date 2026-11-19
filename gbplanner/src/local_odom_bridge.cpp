// Ground-truth odometry for the LARICS position controller and tracker.
//
// The paper assumes perfect localisation, and the planner and map use Gazebo
// ground truth (/<ns>/odometry, world frame). By default the LARICS stack
// (pid_carrot, topp_tracker) flies on ArduPilot's EKF estimate
// (mavros/global_position/local), which in SITL drifts ~1 m from ground truth
// through simulated GPS noise. The UAV then stops short of planned waypoints,
// so PCI never sees the trajectory finish, and it clips walls in narrow
// passages. This node republishes ground truth in the controller's local frame,
// using the same transform as trajectory_transformer:
//   local = Rz(yaw) * (world - offset)
// Velocity follows mavros's conventions: horizontal in the local ENU frame,
// vertical with mavros's inverted sign (see below).
//
// Private params: ~robot (e.g. red), ~offset_x/y/z, ~yaw_deg
// Publishes: /<robot>/odometry_local

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <cmath>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_odom_bridge");
  ros::NodeHandle nh, pnh("~");
  std::string robot;
  if (!pnh.getParam("robot", robot)) {
    ROS_FATAL("~robot not set");
    return 1;
  }
  double ox, oy, oz, yaw_deg;
  pnh.param("offset_x", ox, 0.0);
  pnh.param("offset_y", oy, 0.0);
  pnh.param("offset_z", oz, 0.0);
  pnh.param("yaw_deg", yaw_deg, 0.0);
  const double yaw = yaw_deg * M_PI / 180.0;
  const tf::Quaternion q_rot = tf::createQuaternionFromYaw(yaw);
  const tf::Matrix3x3 R_rot(q_rot);

  ros::Publisher pub =
      nh.advertise<nav_msgs::Odometry>("/" + robot + "/odometry_local", 10);
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(
      "/" + robot + "/odometry", 10,
      [&](const nav_msgs::Odometry::ConstPtr& m) {
        nav_msgs::Odometry o = *m;
        o.header.frame_id = "map";
        o.child_frame_id = robot + "/base_link";
        // Position.
        tf::Vector3 p(m->pose.pose.position.x - ox, m->pose.pose.position.y - oy,
                      m->pose.pose.position.z - oz);
        p = R_rot * p;
        o.pose.pose.position.x = p.x();
        o.pose.pose.position.y = p.y();
        o.pose.pose.position.z = p.z();
        // Orientation.
        tf::Quaternion q;
        tf::quaternionMsgToTF(m->pose.pose.orientation, q);
        tf::quaternionTFToMsg((q_rot * q).normalized(), o.pose.pose.orientation);
        // Linear velocity: Gazebo gives it in the body frame -> local ENU.
        const tf::Matrix3x3 R_body(q);
        tf::Vector3 v(m->twist.twist.linear.x, m->twist.twist.linear.y,
                      m->twist.twist.linear.z);
        v = R_rot * (R_body * v);
        o.twist.twist.linear.x = v.x();
        o.twist.twist.linear.y = v.y();
        // mavros/global_position/local reports vertical velocity with the
        // opposite sign (positive = descending) in this ArduPilot setup, and
        // the LARICS altitude controller is tuned to that convention; a
        // physically correct sign made the UAV overshoot by ~2 m (measured).
        o.twist.twist.linear.z = -v.z();
        // Angular velocity stays in the body frame.
        pub.publish(o);
      });
  ROS_INFO("[%s] ground-truth local odometry: offset (%.2f, %.2f, %.2f), yaw %.1f deg",
           robot.c_str(), ox, oy, oz, yaw_deg);
  ros::spin();
  return 0;
}
