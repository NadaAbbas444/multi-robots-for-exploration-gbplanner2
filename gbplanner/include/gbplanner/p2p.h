// #ifndef P2P_H
// #define P2P_H

// #include <ros/ros.h>
// #include <unordered_map>
// #include <string>
// #include <vector>
// #include "planner_msgs/FrontierRequest.h"
// #include "geometry_msgs/Pose.h"
// #include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

// class FrontierAssigner {
// public:
//     FrontierAssigner(ros::NodeHandle& nh);
//     bool handleFrontierRequest(planner_msgs::FrontierRequest::Request &req, planner_msgs::FrontierRequest::Response &res);
//     void publishVisualizationMarkers();

// private:
//     std::unordered_map<std::string, geometry_msgs::Pose> last_assigned_frontiers;
//     ros::NodeHandle nh_;
//     ros::ServiceServer service_;
//     ros::Publisher Marker_central_comm;
//     std::map<std::string, std::vector<geometry_msgs::Pose>> assigned_frontiers_history;

//     double communication_radius_min_;
//     double communication_radius_max_;
//     double central_radius_;
//     geometry_msgs::Pose central_unit_pose_;

//     double calculateDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
//     visualization_msgs::Marker createMarker(const std::string& ns, int id, int type, double scale, const geometry_msgs::Pose& position, double r, double g, double b);
// };

// #endif // P2P_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <planner_msgs/FrontierRequest.h>
#include <planner_msgs/GetFrontierGains.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <vector>
#include <unordered_map>

class FrontierAssigner {
public:
    FrontierAssigner(ros::NodeHandle& nh);
    bool handleFrontierRequest(planner_msgs::FrontierRequest::Request &req, planner_msgs::FrontierRequest::Response &res);
    void publishVisualizationMarkers();

private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    ros::Publisher Marker_central_comm;
    std::unordered_map<std::string, geometry_msgs::Pose> last_assigned_frontiers;

    std::map<std::string, std::vector<geometry_msgs::Pose>> assigned_frontiers_history;
    geometry_msgs::Pose central_unit_pose_;
    double communication_radius_min_;
    double communication_radius_max_;
    double central_radius_;

    double calculateFrontierGain(const geometry_msgs::Pose& frontier);
    double calculateDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
    visualization_msgs::Marker createMarker(const std::string& ns, int id, int type, double scale, const geometry_msgs::Pose& position, double r, double g, double b);

};




