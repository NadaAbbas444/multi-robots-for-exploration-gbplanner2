// Bridges planner trajectories (Gazebo world frame) to the LARICS
// uav_ros_tracker of the SAME robot (its ArduPilot local frame).
//
// One instance per robot, in that robot's namespace. Parameters (private):
//   ~robot      namespace without slash, e.g. "red"        (required)
//   ~offset_x   world x of the robot's local-frame origin  [m]  (spawn x)
//   ~offset_y   world y of the robot's local-frame origin  [m]  (spawn y)
//   ~offset_z   world z of the robot's local-frame origin  [m]  (default 0)
//   ~yaw_deg    rotation from world to local frame [deg]        (default 0)
//
// local = Rz(yaw) * (world - offset). The commanded heading is rotated too.
// Forwarded streams: command/trajectory, inspection_trajectory, near_obstacle
// -> /<robot>/tracker/input_trajectory.
//
// (The previous version cross-wired red->blue and blue->red, and ran twice.)

#include <ros/ros.h>
#include <tf/tf.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <string>
#include <vector>

class TrajectoryTransformer {
 public:
  TrajectoryTransformer(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    if (!pnh.getParam("robot", robot_)) {
      ROS_FATAL("~robot not set");
      ros::shutdown();
      return;
    }
    pnh.param("offset_x", ox_, 0.0);
    pnh.param("offset_y", oy_, 0.0);
    pnh.param("offset_z", oz_, 0.0);
    double yaw_deg;
    pnh.param("yaw_deg", yaw_deg, 0.0);
    yaw_ = yaw_deg * M_PI / 180.0;

    const std::string ns = "/" + robot_;
    pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        ns + "/tracker/input_trajectory", 10);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(
        ns + "/visualization_path_local", 10);
    for (const std::string& in :
         {"/command/trajectory", "/inspection_trajectory", "/near_obstacle"}) {
      subs_.push_back(nh.subscribe(ns + in, 10, &TrajectoryTransformer::cb, this));
    }
    ROS_INFO("[%s] world->local: offset (%.2f, %.2f, %.2f), yaw %.1f deg",
             robot_.c_str(), ox_, oy_, oz_, yaw_deg);
  }

 private:
  void cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
    trajectory_msgs::MultiDOFJointTrajectory out = *msg;
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.ns = robot_;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.1;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.pose.orientation.w = 1.0;
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    const tf::Quaternion q_rot = tf::createQuaternionFromYaw(yaw_);
    for (auto& pt : out.points) {
      for (auto& t : pt.transforms) {
        const double dx = t.translation.x - ox_, dy = t.translation.y - oy_;
        t.translation.x = c * dx - s * dy;
        t.translation.y = s * dx + c * dy;
        t.translation.z -= oz_;
        tf::Quaternion q;
        tf::quaternionMsgToTF(t.rotation, q);
        if (q.length2() < 1e-9) q.setValue(0, 0, 0, 1);
        tf::quaternionTFToMsg((q_rot * q).normalized(), t.rotation);
        geometry_msgs::Point p;  // marker drawn back in world for checking
        p.x = t.translation.x;
        p.y = t.translation.y;
        p.z = t.translation.z;
        m.points.push_back(p);
      }
    }
    pub_.publish(out);
    marker_pub_.publish(m);
  }

  std::string robot_;
  double ox_, oy_, oz_, yaw_;
  ros::Publisher pub_, marker_pub_;
  std::vector<ros::Subscriber> subs_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_transformer");
  ros::NodeHandle nh, pnh("~");
  TrajectoryTransformer tt(nh, pnh);
  ros::spin();
  return 0;
}
