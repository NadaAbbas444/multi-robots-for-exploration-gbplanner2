// Re-stamps a point cloud with a namespaced frame id.
//
// The LARICS kopterworx LiDAR (LiDAR-X macro) publishes every robot's cloud
// in the same un-prefixed frame "velodyne", which is not connected to any
// robot in TF. This relay publishes the cloud as "<ns>/velodyne"; robot.launch
// adds the matching static transforms from "<ns>/base_link".
//   ~input   (default: velodyne_points)        ~output (default: velodyne_points_ns)
//   ~frame_id (required, e.g. red/velodyne)

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_frame_relay");
  ros::NodeHandle nh, pnh("~");
  std::string in, out, frame;
  pnh.param<std::string>("input", in, "velodyne_points");
  pnh.param<std::string>("output", out, "velodyne_points_ns");
  if (!pnh.getParam("frame_id", frame)) {
    ROS_FATAL("~frame_id is required");
    return 1;
  }
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(out, 5);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      in, 5, [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        sensor_msgs::PointCloud2 c = *msg;
        c.header.frame_id = frame;
        pub.publish(c);
      });
  ros::spin();
  return 0;
}
