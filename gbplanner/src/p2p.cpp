#include "gbplanner/p2p.h"
#include <fstream>
#include <cmath>
#include <limits>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <unordered_map>

FrontierAssigner::FrontierAssigner(ros::NodeHandle& nh) : nh_(nh) {
    if (!nh_.getParam("frontier_assignment_node/communication_radius_min", communication_radius_min_)) {
        ROS_WARN("Failed to get 'communication_radius_min' parameter. Setting to default value of 10.0");
        communication_radius_min_ = 10.0;
    }
    if (!nh_.getParam("frontier_assignment_node/communication_radius_max", communication_radius_max_)) {
        ROS_WARN("Failed to get 'communication_radius_max' parameter. Setting to default value of 100.0");
        communication_radius_max_ = 200.0;
    }
    if (!nh_.getParam("frontier_assignment_node/central_radius", central_radius_)) {
        ROS_WARN("Failed to get 'central_radius' parameter. Setting to default value of 50.0");
        central_radius_ = 50.0;
    }
    std::vector<double> default_pose = {0.0, 0.0, 0.0};
    std::vector<double> pose;
    if (!nh_.getParam("frontier_assignment_node/Central_Unit_pose", pose)) {
        ROS_WARN("Failed to get 'Central_Unit_pose'. Setting to default [0.0, 0.0, 0.0]");
        pose = default_pose;
    }
    central_unit_pose_.position.x = pose[0];
    central_unit_pose_.position.y = pose[1];
    central_unit_pose_.position.z = pose[2];

    service_ = nh_.advertiseService("assign_frontier", &FrontierAssigner::handleFrontierRequest, this);
    Marker_central_comm = nh_.advertise<visualization_msgs::Marker>("visualization_marker_central_comm", 10);
    ROS_INFO("Frontier Assignment Service is ready.");
}

bool FrontierAssigner::handleFrontierRequest(planner_msgs::FrontierRequest::Request &req, planner_msgs::FrontierRequest::Response &res) {
    auto& last_frontier = last_assigned_frontiers[req.robot_id];
    double best_gain = -std::numeric_limits<double>::infinity();
    int best_id = -1;
    double highest_gain = -std::numeric_limits<double>::infinity();
    int highest_gain_id = -1;

    ROS_INFO_STREAM("Received request from robot: " << req.robot_id);

    for (size_t i = 0; i < req.frontiers.poses.size(); ++i) {
        double distance = calculateDistance(req.frontiers.poses[i], last_frontier);
        double gain = req.gains[i];

        if (gain > highest_gain) {
            highest_gain = gain;
            highest_gain_id = i;
        }

        if (distance >= communication_radius_min_ && distance <= communication_radius_max_ && gain > best_gain) {
            best_gain = gain;
            best_id = i;
        }
    }

    if (best_id != -1) {
        res.assigned_frontier_id = req.ids[best_id];
        res.success = true;
        ROS_INFO("Assigned Frontier Pose: x = %.2f, y = %.2f, z = %.2f",
                 req.frontiers.poses[best_id].position.x,
                 req.frontiers.poses[best_id].position.y,
                 req.frontiers.poses[best_id].position.z);
    } else {
        res.assigned_frontier_id = req.ids[highest_gain_id];
        res.success = true;
        ROS_WARN("No suitable frontier within communication range. Assigning highest gain frontier instead.");
        best_id = highest_gain_id; // Update best_id to highest_gain_id for file writing
    }

    return true;
}


// bool FrontierAssigner::handleFrontierRequest(planner_msgs::FrontierRequest::Request &req, planner_msgs::FrontierRequest::Response &res) {
//     double best_gain = -std::numeric_limits<double>::infinity();
//     int best_id = -1;
//     double highest_gain = -std::numeric_limits<double>::infinity();
//     int highest_gain_id = -1;

//     ROS_INFO_STREAM("Received request from robot: " << req.robot_id);

//     for (size_t i = 0; i < req.frontiers.poses.size(); ++i) {
//         const auto& current_pose = req.frontiers.poses[i];
//         double gain = req.gains[i];
//         bool is_far_enough = true;

//         // Compare against the history of all robots
//         for (const auto& [robot_id, history] : assigned_frontiers_history) {
//             for (const auto& previous_pose : history) {
//                 if (calculateDistance(current_pose, previous_pose) < 5.0) {
//                     is_far_enough = false;
//                     break;
//                 }
//             }
//             if (!is_far_enough) {
//                 break;
//             }
//         }

//         if (!is_far_enough) {
//             continue;
//         }

//         if (gain > highest_gain) {
//             highest_gain = gain;
//             highest_gain_id = i;
//         }

//         if (gain > best_gain && calculateDistance(current_pose, req.frontiers.poses.back()) >= communication_radius_min_ && calculateDistance(current_pose, req.frontiers.poses.back()) <= communication_radius_max_) {
//             best_gain = gain;
//             best_id = i;
//         }
//     }

//     if (best_id != -1) {
//         res.assigned_frontier_id = req.ids[best_id];
//         res.success = true;
//         assigned_frontiers_history[req.robot_id].push_back(req.frontiers.poses[best_id]); // Update history with the newly assigned frontier
//         ROS_INFO("Assigned Frontier Pose: x = %.2f, y = %.2f, z = %.2f",
//                  req.frontiers.poses[best_id].position.x,
//                  req.frontiers.poses[best_id].position.y,
//                  req.frontiers.poses[best_id].position.z);
//     } else {
//         res.assigned_frontier_id = req.ids[highest_gain_id];
//         res.success = true;
//         assigned_frontiers_history[req.robot_id].push_back(req.frontiers.poses[highest_gain_id]); // Update history with the highest gain frontier
//         ROS_WARN("No suitable frontier within communication range. Assigning highest gain frontier instead.");
//         best_id = highest_gain_id; // Update best_id to highest_gain_id for file writing
//     }

//     return true;
// }



double FrontierAssigner::calculateDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    double dz = p1.position.z - p2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void FrontierAssigner::publishVisualizationMarkers() {
    visualization_msgs::Marker central_unit = createMarker("space_visualization", 0, visualization_msgs::Marker::CUBE, 1.0, central_unit_pose_, 0, 0, 1);
    visualization_msgs::Marker communication_circle = createMarker("space_visualization", 1, visualization_msgs::Marker::CYLINDER, 2 * central_radius_, central_unit_pose_, 0, 1, 0);

    Marker_central_comm.publish(central_unit);
    Marker_central_comm.publish(communication_circle);
}

visualization_msgs::Marker FrontierAssigner::createMarker(const std::string& ns, int id, int type, double scale, const geometry_msgs::Pose& position, double r, double g, double b) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = position.position; // Make sure position is correctly assigned
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = 0.1; // Adjust z scale for visibility
    marker.color.a = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "p2p");
    ros::NodeHandle nh;
    FrontierAssigner assigner(nh);
    ros::Rate r(1);

    while (ros::ok()) {
        assigner.publishVisualizationMarkers();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
