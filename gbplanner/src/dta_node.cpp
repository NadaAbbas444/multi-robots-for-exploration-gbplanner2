#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <cmath>
#include "planner_msgs/DistanceAndRole.h"

class DistanceRoleService {
public:
    DistanceRoleService(ros::NodeHandle& nh) : nh_(nh) {
        if (!nh_.getParam("role", role_)) {
            ROS_ERROR("Failed to get 'role' parameter.");
            ros::shutdown();
        }

        if (!nh_.getParam("namespace", namespace_)) {
            ROS_ERROR("Failed to get 'namespace' parameter.");
            ros::shutdown();
        }

        // Initialize service server
        service_ = nh_.advertiseService("compute_distance_and_role", &DistanceRoleService::computeDistance, this);
        ROS_INFO("Service for computing distance and role is now available.");
    }

    bool computeDistance(planner_msgs::DistanceAndRole::Request &req,
                         planner_msgs::DistanceAndRole::Response &res) {
        // Construct the odometry topic based on the namespace
        std::string odometry_topic = "/" + namespace_ + "/odometry";

        // Temporary subscriber to get the latest odometry
        geometry_msgs::Point current_pose;
        ros::Subscriber temp_sub = nh_.subscribe<nav_msgs::Odometry>(odometry_topic, 1, 
            [&current_pose](const nav_msgs::Odometry::ConstPtr& msg) {
                current_pose = msg->pose.pose.position;
            });

        // Wait for one message to ensure the pose is updated
        ros::Rate rate(10); // Adjust the rate as needed
        while (ros::ok() && !temp_sub.getNumPublishers()) {
            ros::spinOnce();
            rate.sleep();
        }
        ros::spinOnce(); // Ensure callback is processed

        // Unsubscribe after receiving the first message
        temp_sub.shutdown();

        // Calculate distance from current_pose to req.point
        double dx = req.point.x - current_pose.x;
        double dy = req.point.y - current_pose.y;
        double dz = req.point.z - current_pose.z;
        res.distance = sqrt(dx*dx + dy*dy + dz*dz);

        // Return the role
        res.role = role_;

        return true;
    }

private:
    ros::NodeHandle nh_;
    std::string namespace_;
    std::string role_;
    ros::ServiceServer service_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "dta_node");
    ros::NodeHandle nh("~");
    DistanceRoleService drs(nh);
    ros::spin();
    return 0;
}
