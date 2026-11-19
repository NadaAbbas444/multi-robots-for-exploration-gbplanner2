#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <planner_semantic_msgs/SemanticPoint.h>
#include <planner_semantic_msgs/SemanticClass.h>
#include <planner_msgs/DistanceAndRole.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <vector>
#include <cmath>
#include <map>

struct SemanticData {
    geometry_msgs::Point point;
    int description;
};

class DynamicTaskAllocator {
public:
    DynamicTaskAllocator() : nh_("~"), done_inspection_red_(false), done_inspection_blue_(false) {
        namespace_red_ = "red";
        namespace_blue_ = "blue";

        // Service clients
        my_distance_service_red_ = nh_.serviceClient<planner_msgs::DistanceAndRole>("/red/dta_node/compute_distance_and_role");
        my_distance_service_blue_ = nh_.serviceClient<planner_msgs::DistanceAndRole>("/blue/dta_node/compute_distance_and_role");
        toggle_planning_client_red_ = nh_.serviceClient<std_srvs::SetBool>("/red/gbplanner/toggle_planning");
        toggle_planning_client_blue_ = nh_.serviceClient<std_srvs::SetBool>("/blue/gbplanner/toggle_planning");
        stop_planning_client_red_ = nh_.serviceClient<std_srvs::Trigger>("/red/planner_control_interface/std_srvs/stop");
        stop_planning_client_blue_ = nh_.serviceClient<std_srvs::Trigger>("/blue/planner_control_interface/std_srvs/stop");
        automatic_planning_client_red_ = nh_.serviceClient<std_srvs::Trigger>("/red/planner_control_interface/std_srvs/automatic_planning");
        automatic_planning_client_blue_ = nh_.serviceClient<std_srvs::Trigger>("/blue/planner_control_interface/std_srvs/automatic_planning");

        // Subscribers
        done_inspection_sub_red_ = nh_.subscribe("/red/done_inspection", 1, &DynamicTaskAllocator::doneInspectionCallbackRed, this);
        done_inspection_sub_blue_ = nh_.subscribe("/blue/done_inspection", 1, &DynamicTaskAllocator::doneInspectionCallbackBlue, this);
        semantic_sub_red_ = nh_.subscribe("/red/previous_semantic_points", 1, &DynamicTaskAllocator::semanticCallback, this);
        semantic_sub_blue_ = nh_.subscribe("/blue/previous_semantic_points", 1, &DynamicTaskAllocator::semanticCallback, this);

        // Publishers
        semantic_pub_red_ = nh_.advertise<planner_semantic_msgs::SemanticPoint>("/red/semantic_location", 10);
        semantic_pub_blue_ = nh_.advertise<planner_semantic_msgs::SemanticPoint>("/blue/semantic_location", 10);
        timer_ = nh_.createTimer(ros::Duration(500), &DynamicTaskAllocator::allocateTasks, this);
    }

private:
    ros::NodeHandle nh_;
    std::vector<SemanticData> points_;
    std::vector<SemanticData> red_tasks_;
    std::vector<SemanticData> blue_tasks_;
    bool done_inspection_red_;
    bool done_inspection_blue_;

    std::string namespace_red_;
    std::string namespace_blue_;

    ros::ServiceClient my_distance_service_red_;
    ros::ServiceClient my_distance_service_blue_;
    ros::ServiceClient toggle_planning_client_red_;
    ros::ServiceClient toggle_planning_client_blue_;
    ros::ServiceClient stop_planning_client_red_;
    ros::ServiceClient stop_planning_client_blue_;
    ros::ServiceClient automatic_planning_client_red_;
    ros::ServiceClient automatic_planning_client_blue_;
    ros::Subscriber done_inspection_sub_red_;
    ros::Subscriber done_inspection_sub_blue_;
    ros::Subscriber semantic_sub_red_;
    ros::Subscriber semantic_sub_blue_;
    ros::Publisher semantic_pub_red_;
    ros::Publisher semantic_pub_blue_;
    ros::Timer timer_;

    void semanticCallback(const planner_semantic_msgs::SemanticPoint::ConstPtr& msg) {
        geometry_msgs::Point point;
        point.x = msg->point.x;
        point.y = msg->point.y;
        point.z = msg->point.z;
        int description = msg->type.value;

        points_.push_back({point, description});
        ROS_INFO_STREAM("Collected point: (" << point.x << ", " << point.y << ", " << point.z << ") with description: " << description);
    }

    void doneInspectionCallbackRed(const std_msgs::Bool::ConstPtr& msg) {
        done_inspection_red_ = msg->data;
        ROS_INFO_STREAM("Received done_inspection from red: " << done_inspection_red_);
        ROS_INFO_STREAM("Current red_tasks_ size: " << red_tasks_.size());
        if (done_inspection_red_ && red_tasks_.empty()) {
            reactivatePlannerIfNoTasks(red_tasks_, toggle_planning_client_red_, stop_planning_client_red_, automatic_planning_client_red_);
        }
    }

    void doneInspectionCallbackBlue(const std_msgs::Bool::ConstPtr& msg) {
        done_inspection_blue_ = msg->data;
        ROS_INFO_STREAM("Received done_inspection from blue: " << done_inspection_blue_);
        if (done_inspection_blue_ && blue_tasks_.empty()) {
            reactivatePlannerIfNoTasks(blue_tasks_, toggle_planning_client_blue_, stop_planning_client_blue_, automatic_planning_client_blue_);
        }
    }

    double distance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) {
        return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
    }

    std::map<int, std::map<std::string, double>> calculateCosts() {
        std::map<int, std::map<std::string, double>> costs;
        for (size_t idx = 0; idx < points_.size(); ++idx) {
            const auto& point = points_[idx].point;
            planner_msgs::DistanceAndRole srv;
            srv.request.point = point;

            if (my_distance_service_red_.call(srv)) {
                costs[idx]["red"] = srv.response.distance;
            } else {
                ROS_ERROR("Failed to call red distance service");
            }

            if (my_distance_service_blue_.call(srv)) {
                costs[idx]["blue"] = srv.response.distance;
            } else {
                ROS_ERROR("Failed to call blue distance service");
            }
        }
        return costs;
    }

    void filterTasksByProximity(std::vector<SemanticData>& tasks, double proximity_threshold) {
        std::vector<SemanticData> filtered;
        for (auto& task : tasks) {
            bool is_too_close = std::any_of(filtered.begin(), filtered.end(), [&task, proximity_threshold, this](const SemanticData& other) {
                return distance(task.point, other.point) < proximity_threshold;
            });
            if (!is_too_close) {
                filtered.push_back(task);
            }
        }
        tasks.swap(filtered);
    }

    void allocateTasks(const ros::TimerEvent&) {
        if (points_.empty()) {
            ROS_INFO("No points to allocate.");
            return;
        }

        filterTasksByProximity(points_, 6.0);  // Example threshold of 8.0 units

        // Stop the planners
        std_srvs::SetBool toggle_srv;
        toggle_srv.request.data = false;
        if (toggle_planning_client_red_.call(toggle_srv) && toggle_planning_client_blue_.call(toggle_srv)) {
            ROS_INFO("Planners stopped.");
        } else {
            ROS_ERROR("Failed to stop planners.");
            return;
        }

        auto costs = calculateCosts();
        red_tasks_.clear();
        blue_tasks_.clear();

        for (const auto& [idx, cost] : costs) {
            if (cost.at("red") < cost.at("blue")) {
                red_tasks_.push_back(points_[idx]);
            } else {
                blue_tasks_.push_back(points_[idx]);
            }
        }

        bool reactivated_red = false;
        bool reactivated_blue = false;

        // Check if there are no tasks assigned to the red robot
        if (red_tasks_.empty()) {
            ROS_INFO("No tasks assigned to red robot. Reactivating red planner.");
            reactivatePlannerIfNoTasks(red_tasks_, toggle_planning_client_red_, stop_planning_client_red_, automatic_planning_client_red_);
            reactivated_red = true;
        }
        else{
            for (const auto& task : red_tasks_) {
                ROS_INFO_STREAM("Task in red: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
            }
        }

        // Check if there are no tasks assigned to the blue robot
        if (blue_tasks_.empty()) {
            ROS_INFO("No tasks assigned to blue robot. Reactivating blue planner.");
            reactivatePlannerIfNoTasks(blue_tasks_, toggle_planning_client_blue_, stop_planning_client_blue_, automatic_planning_client_blue_);
            reactivated_blue = true;
        }

        else{
            for (const auto& task : blue_tasks_) {
                ROS_INFO_STREAM("Task in blue: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
            }
        }

        // If both robots have no tasks and both planners were reactivated, clear points and return
        if (reactivated_red && reactivated_blue) {
            points_.clear();
            return;
        }

        // // Log and publish tasks only if tasks were assigned
        // if (!red_tasks_.empty()) {
        //     logAndPublishTasks(red_tasks_, "red");
        // }
        // if (!blue_tasks_.empty()) {
        //     logAndPublishTasks(blue_tasks_, "blue");
        // }
        
        logAndPublishTasks(red_tasks_, blue_tasks_);

        // Print assigned tasks to ROS logs
        if (!red_tasks_.empty()) {
            ROS_INFO_STREAM("Assigned tasks to red robot:");
            for (const auto& task : red_tasks_) {
                ROS_INFO_STREAM("Task at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
            }
        }

        if (!blue_tasks_.empty()) {
            ROS_INFO_STREAM("Assigned tasks to blue robot:");
            for (const auto& task : blue_tasks_) {
                ROS_INFO_STREAM("Task at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
            }
        }

        points_.clear();  // Clear the list of points after allocation

        // Debugging before clearing
        ROS_INFO_STREAM("Before clearing, red_tasks_ size: " << red_tasks_.size());
        ROS_INFO_STREAM("Before clearing, blue_tasks_ size: " << blue_tasks_.size());

        // Ensure tasks are cleared after processing
        red_tasks_.clear();
        blue_tasks_.clear();

        // Debugging after clearing
        ROS_INFO_STREAM("After clearing, red_tasks_ size: " << red_tasks_.size());
        ROS_INFO_STREAM("After clearing, blue_tasks_ size: " << blue_tasks_.size());

        // Reactivate planners if no tasks are assigned (if they weren't reactivated before)
        if (!reactivated_red) {
            reactivatePlannerIfNoTasks(red_tasks_, toggle_planning_client_red_, stop_planning_client_red_, automatic_planning_client_red_);
        }
        if (!reactivated_blue) {
            reactivatePlannerIfNoTasks(blue_tasks_, toggle_planning_client_blue_, stop_planning_client_blue_, automatic_planning_client_blue_);
        }
    }


    // void logAndPublishTasks(const std::vector<SemanticData>& tasks, const std::string& robot_namespace) {
    //     if (tasks.empty()) {
    //         ROS_INFO_STREAM("No tasks assigned to " << robot_namespace << " robot. Enabling planner to continue exploring.");
    //     } else {
    //         ROS_INFO_STREAM("Assigned tasks to " << robot_namespace << " robot:");
    //         for (const auto& task : tasks) {
    //             ROS_INFO_STREAM("Task at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
                
    //             // Reset the done_inspection flag for the respective robot
    //             if (robot_namespace == "red") {
    //                 done_inspection_red_ = false;
    //             } else {
    //                 done_inspection_blue_ = false;
    //             }

    //             publishTask(task, robot_namespace);

    //             // Wait for the done_inspection flag to be set to true
    //             ros::Rate rate(10); // Check at 10 Hz
    //             while (ros::ok() && ((robot_namespace == "red" && !done_inspection_red_) ||
    //                                 (robot_namespace == "blue" && !done_inspection_blue_))) {
    //                 ros::spinOnce();
    //                 rate.sleep();
    //             }
    //         }
    //     }
    // }


    void logAndPublishTasks(const std::vector<SemanticData>& red_tasks, const std::vector<SemanticData>& blue_tasks) {
        size_t red_index = 0;
        size_t blue_index = 0;
        
        while (red_index < red_tasks.size() || blue_index < blue_tasks.size()) {
            // Assign red task if there are remaining tasks
            if (red_index < red_tasks.size()) {
                const auto& task = red_tasks[red_index];
                ROS_INFO_STREAM("Assigning task to red robot at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
                done_inspection_red_ = false;
                publishTask(task, "red");
            }

            // Assign blue task if there are remaining tasks
            if (blue_index < blue_tasks.size()) {
                const auto& task = blue_tasks[blue_index];
                ROS_INFO_STREAM("Assigning task to blue robot at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
                done_inspection_blue_ = false;
                publishTask(task, "blue");
            }

            // Wait for the respective robot to finish its task
            ros::Rate rate(10); // Check at 10 Hz
            while (ros::ok() && ((red_index < red_tasks.size() && !done_inspection_red_) || 
                                (blue_index < blue_tasks.size() && !done_inspection_blue_))) {
                ros::spinOnce();
                rate.sleep();
            }

            // Move to the next task in the respective list
            if (red_index < red_tasks.size() && done_inspection_red_) {
                ++red_index;
            }
            if (blue_index < blue_tasks.size() && done_inspection_blue_) {
                ++blue_index;
            }
        }
    }



    void reactivatePlannerIfNoTasks(const std::vector<SemanticData>& tasks, ros::ServiceClient& planning_client, ros::ServiceClient& stop_client, ros::ServiceClient& automatic_client) {
        if (tasks.empty()) {
            std_srvs::SetBool toggle_srv;
            toggle_srv.request.data = true; // Enable planner
            if (planning_client.call(toggle_srv)) {
                ROS_INFO("Planner restarted for exploration.");

                // Trigger the stop and automatic planning services
                std_srvs::Trigger trigger_srv;
                
                if (stop_client.call(trigger_srv) && trigger_srv.response.success) {
                    ROS_INFO("Stop service called successfully.");
                } else {
                    ROS_WARN("Failed to call stop service.");
                }

                if (automatic_client.call(trigger_srv) && trigger_srv.response.success) {
                    ROS_INFO("Automatic planning service called successfully.");
                } else {
                    ROS_WARN("Failed to call automatic planning service.");
                }
            } else {
                ROS_ERROR("Failed to restart planner.");
            }
        }
    }

    void publishTask(const SemanticData& task, const std::string& robot_namespace) {
        planner_semantic_msgs::SemanticPoint msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.type.value = task.description;
        msg.point.x = task.point.x;
        msg.point.y = task.point.y;
        msg.point.z = task.point.z;

        if (robot_namespace == "red") {
            semantic_pub_red_.publish(msg);
            ROS_INFO_STREAM("Published task for red robot at point: (" << msg.point.x << ", " << msg.point.y << ", " << msg.point.z << ")");
        } else {
            semantic_pub_blue_.publish(msg);
            ROS_INFO_STREAM("Published task for blue robot at point: (" << msg.point.x << ", " << msg.point.y << ", " << msg.point.z << ")");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_auction");
    DynamicTaskAllocator allocator;
    ros::spin();
    return 0;
}

// ________________ for one robot for collecting results _______________________

// #include <ros/ros.h>
// #include <geometry_msgs/Point.h>
// #include <std_msgs/Bool.h>
// #include <planner_semantic_msgs/SemanticPoint.h>
// #include <planner_msgs/DistanceAndRole.h>
// #include <std_srvs/SetBool.h>
// #include <std_srvs/Trigger.h>
// #include <vector>
// #include <cmath>
// #include <map>

// struct SemanticData {
//     geometry_msgs::Point point;
//     int description;
// };

// class DynamicTaskAllocator {
// public:
//     DynamicTaskAllocator() : nh_("~"), done_inspection_red_(false) {
//         namespace_red_ = "red";

//         // Service clients
//         my_distance_service_red_ = nh_.serviceClient<planner_msgs::DistanceAndRole>("/red/dta_node/compute_distance_and_role");
//         toggle_planning_client_red_ = nh_.serviceClient<std_srvs::SetBool>("/red/gbplanner/toggle_planning");
//         stop_planning_client_red_ = nh_.serviceClient<std_srvs::Trigger>("/red/planner_control_interface/std_srvs/stop");
//         automatic_planning_client_red_ = nh_.serviceClient<std_srvs::Trigger>("/red/planner_control_interface/std_srvs/automatic_planning");

//         // Subscribers
//         done_inspection_sub_red_ = nh_.subscribe("/red/done_inspection", 1, &DynamicTaskAllocator::doneInspectionCallbackRed, this);
//         semantic_sub_red_ = nh_.subscribe("/red/previous_semantic_points", 1, &DynamicTaskAllocator::semanticCallback, this);

//         // Publisher
//         semantic_pub_red_ = nh_.advertise<planner_semantic_msgs::SemanticPoint>("/red/semantic_location", 10);

//         // Timer for task allocation
//         timer_ = nh_.createTimer(ros::Duration(10), &DynamicTaskAllocator::allocateTasks, this);
//     }

// private:
//     ros::NodeHandle nh_;
//     std::vector<SemanticData> points_;
//     std::vector<SemanticData> red_tasks_;
//     bool done_inspection_red_;
//     std::string namespace_red_;

//     ros::ServiceClient my_distance_service_red_;
//     ros::ServiceClient toggle_planning_client_red_;
//     ros::ServiceClient stop_planning_client_red_;
//     ros::ServiceClient automatic_planning_client_red_;

//     ros::Subscriber done_inspection_sub_red_;
//     ros::Subscriber semantic_sub_red_;

//     ros::Publisher semantic_pub_red_;

//     ros::Timer timer_;

//     void semanticCallback(const planner_semantic_msgs::SemanticPoint::ConstPtr& msg) {
//         geometry_msgs::Point point;
//         point.x = msg->point.x;
//         point.y = msg->point.y;
//         point.z = msg->point.z;
//         int description = msg->type.value;

//         points_.push_back({point, description});
//         ROS_INFO_STREAM("Collected point: (" << point.x << ", " << point.y << ", " << point.z << ") with description: " << description);
//     }

//     void doneInspectionCallbackRed(const std_msgs::Bool::ConstPtr& msg) {
//         done_inspection_red_ = msg->data;
//         ROS_INFO_STREAM("Received done_inspection from red: " << done_inspection_red_);
//         ROS_INFO_STREAM("Current red_tasks_ size: " << red_tasks_.size());
//         if (done_inspection_red_ && red_tasks_.empty()) {
//             reactivatePlannerIfNoTasks(red_tasks_, toggle_planning_client_red_, stop_planning_client_red_, automatic_planning_client_red_);
//         }
//     }

//     double distance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) {
//         return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
//     }

//     std::map<int, double> calculateCosts() {
//         std::map<int, double> costs;
//         for (size_t idx = 0; idx < points_.size(); ++idx) {
//             const auto& point = points_[idx].point;
//             planner_msgs::DistanceAndRole srv;
//             srv.request.point = point;

//             if (my_distance_service_red_.call(srv)) {
//                 costs[idx] = srv.response.distance;
//             } else {
//                 ROS_ERROR("Failed to call red distance service");
//             }
//         }
//         return costs;
//     }

//     void filterTasksByProximity(std::vector<SemanticData>& tasks, double proximity_threshold) {
//         std::vector<SemanticData> filtered;
//         for (auto& task : tasks) {
//             bool is_too_close = std::any_of(filtered.begin(), filtered.end(), [&task, proximity_threshold, this](const SemanticData& other) {
//                 return distance(task.point, other.point) < proximity_threshold;
//             });
//             if (!is_too_close) {
//                 filtered.push_back(task);
//             }
//         }
//         tasks.swap(filtered);
//     }

//     void allocateTasks(const ros::TimerEvent&) {
//         if (points_.empty()) {
//             ROS_INFO("No points to allocate.");
//             return;
//         }

//         filterTasksByProximity(points_, 8.0);  // Example threshold of 8.0 units

//         // Stop the planner
//         std_srvs::SetBool toggle_srv;
//         toggle_srv.request.data = false;
//         if (toggle_planning_client_red_.call(toggle_srv)) {
//             ROS_INFO("Planner stopped.");
//         } else {
//             ROS_ERROR("Failed to stop planner.");
//             return;
//         }

//         auto costs = calculateCosts();
//         red_tasks_.clear();

//         for (const auto& [idx, cost] : costs) {
//             red_tasks_.push_back(points_[idx]);
//         }

//         // Log and publish tasks
//         logAndPublishTasks(red_tasks_, "red");

//         // Print assigned tasks to ROS logs
//         ROS_INFO_STREAM("Assigned tasks to red robot:");
//         for (const auto& task : red_tasks_) {
//             ROS_INFO_STREAM("Task at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
//         }

//         points_.clear();  // Clear the list of points after allocation

//         // Debugging before clearing
//         ROS_INFO_STREAM("Before clearing, red_tasks_ size: " << red_tasks_.size());

//         // Ensure tasks are cleared after processing
//         red_tasks_.clear();

//         // Debugging after clearing
//         ROS_INFO_STREAM("After clearing, red_tasks_ size: " << red_tasks_.size());

//         // Check and reactivate planner if no tasks are assigned
//         reactivatePlannerIfNoTasks(red_tasks_, toggle_planning_client_red_, stop_planning_client_red_, automatic_planning_client_red_);
//     }

//     void logAndPublishTasks(const std::vector<SemanticData>& tasks, const std::string& robot_namespace) {
//         if (tasks.empty()) {
//             ROS_INFO_STREAM("No tasks assigned to " << robot_namespace << " robot. Enabling planner to continue exploring.");
//         } else {
//             ROS_INFO_STREAM("Assigned tasks to " << robot_namespace << " robot:");
//             for (const auto& task : tasks) {
//                 ROS_INFO_STREAM("Task at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
//                 publishTask(task, robot_namespace);
//             }
//         }
//     }

//     void reactivatePlannerIfNoTasks(const std::vector<SemanticData>& tasks, ros::ServiceClient& planning_client, ros::ServiceClient& stop_client, ros::ServiceClient& automatic_client) {
//         if (tasks.empty()) {
//             std_srvs::SetBool toggle_srv;
//             toggle_srv.request.data = true; // Enable planner
//             if (planning_client.call(toggle_srv)) {
//                 ROS_INFO("Planner restarted for exploration.");

//                 // Trigger the stop and automatic planning services
//                 std_srvs::Trigger trigger_srv;
                
//                 if (stop_client.call(trigger_srv) && trigger_srv.response.success) {
//                     ROS_INFO("Stop service called successfully.");
//                 } else {
//                     ROS_WARN("Failed to call stop service.");
//                 }

//                 if (automatic_client.call(trigger_srv) && trigger_srv.response.success) {
//                     ROS_INFO("Automatic planning service called successfully.");
//                 } else {
//                     ROS_WARN("Failed to call automatic planning service.");
//                 }
//             } else {
//                 ROS_ERROR("Failed to restart planner.");
//             }
//         }
//     }

//     void publishTask(const SemanticData& task, const std::string& robot_namespace) {
//         planner_semantic_msgs::SemanticPoint msg;
//         msg.header.stamp = ros::Time::now();
//         msg.header.frame_id = "world";
//         msg.type.value = task.description;
//         msg.point.x = task.point.x;
//         msg.point.y = task.point.y;
//         msg.point.z = task.point.z;

//         semantic_pub_red_.publish(msg);
//         ROS_INFO_STREAM("Published task for red robot at point: (" << msg.point.x << ", " << msg.point.y << ", " << msg.point.z << ")");
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "dynamic_auction");
//     DynamicTaskAllocator allocator;
//     ros::spin();
//     return 0;
// }












//for the plot comment done_inspection and timer at rrg _____________________________________________________________________________________
// #include <ros/ros.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Point32.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Bool.h>
// #include <planner_semantic_msgs/SemanticPoint.h>
// #include <planner_semantic_msgs/SemanticClass.h>
// #include <planner_msgs/DistanceAndRole.h>
// #include <std_srvs/SetBool.h>
// #include <std_srvs/Trigger.h>
// #include <nav_msgs/Odometry.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <vector>
// #include <cmath>
// #include <map>

// struct SemanticData {
//     geometry_msgs::Point point;
//     int description;
// };

// class DynamicTaskAllocator {
// public:
//     DynamicTaskAllocator() : nh_("~"), reached_final_pose_red_(false), reached_final_pose_blue_(false) {
//         namespace_red_ = "red";
//         namespace_blue_ = "blue";

//         // Service clients
//         my_distance_service_red_ = nh_.serviceClient<planner_msgs::DistanceAndRole>("/red/dta_node/compute_distance_and_role");
//         my_distance_service_blue_ = nh_.serviceClient<planner_msgs::DistanceAndRole>("/blue/dta_node/compute_distance_and_role");
//         toggle_planning_client_red_ = nh_.serviceClient<std_srvs::SetBool>("/red/gbplanner/toggle_planning");
//         toggle_planning_client_blue_ = nh_.serviceClient<std_srvs::SetBool>("/blue/gbplanner/toggle_planning");
//         stop_planning_client_red_ = nh_.serviceClient<std_srvs::Trigger>("/red/planner_control_interface/std_srvs/stop");
//         stop_planning_client_blue_ = nh_.serviceClient<std_srvs::Trigger>("/blue/planner_control_interface/std_srvs/stop");
//         automatic_planning_client_red_ = nh_.serviceClient<std_srvs::Trigger>("/red/planner_control_interface/std_srvs/automatic_planning");
//         automatic_planning_client_blue_ = nh_.serviceClient<std_srvs::Trigger>("/blue/planner_control_interface/std_srvs/automatic_planning");

//         // Subscribers
//         semantic_sub_red_ = nh_.subscribe("/red/previous_semantic_points", 1, &DynamicTaskAllocator::semanticCallback, this);
//         semantic_sub_blue_ = nh_.subscribe("/blue/previous_semantic_points", 1, &DynamicTaskAllocator::semanticCallback, this);
//         odometry_sub_red_ = nh_.subscribe("/red/odometry", 1, &DynamicTaskAllocator::odometryCallbackRed, this);
//         odometry_sub_blue_ = nh_.subscribe("/blue/odometry", 1, &DynamicTaskAllocator::odometryCallbackBlue, this);
//         adjusted_points_sub_red_ = nh_.subscribe("/red/inspection_trajectory", 10, &DynamicTaskAllocator::adjustedPointsCallbackRed, this);
//         adjusted_points_sub_blue_ = nh_.subscribe("/blue/inspection_trajectory", 10, &DynamicTaskAllocator::adjustedPointsCallbackBlue, this);

//         // Publishers
//         semantic_pub_red_ = nh_.advertise<planner_semantic_msgs::SemanticPoint>("/red/semantic_location", 10);
//         semantic_pub_blue_ = nh_.advertise<planner_semantic_msgs::SemanticPoint>("/blue/semantic_location", 10);

//         // Timer for task allocation
//         timer_ = nh_.createTimer(ros::Duration(10), &DynamicTaskAllocator::allocateTasks, this);
//     }

// private:
//     ros::NodeHandle nh_;
//     std::vector<SemanticData> points_;
//     std::vector<SemanticData> red_tasks_;
//     std::vector<SemanticData> blue_tasks_;
//     bool reached_final_pose_red_;
//     bool reached_final_pose_blue_;
//     geometry_msgs::Point last_pose_red_;
//     geometry_msgs::Point last_pose_blue_;
//     geometry_msgs::Point target_pose_red_;
//     geometry_msgs::Point target_pose_blue_;

//     std::string namespace_red_;
//     std::string namespace_blue_;

//     ros::ServiceClient my_distance_service_red_;
//     ros::ServiceClient my_distance_service_blue_;
//     ros::ServiceClient toggle_planning_client_red_;
//     ros::ServiceClient toggle_planning_client_blue_;
//     ros::ServiceClient stop_planning_client_red_;
//     ros::ServiceClient stop_planning_client_blue_;
//     ros::ServiceClient automatic_planning_client_red_;
//     ros::ServiceClient automatic_planning_client_blue_;

//     ros::Subscriber semantic_sub_red_;
//     ros::Subscriber semantic_sub_blue_;
//     ros::Subscriber odometry_sub_red_;
//     ros::Subscriber odometry_sub_blue_;
//     ros::Subscriber adjusted_points_sub_red_;
//     ros::Subscriber adjusted_points_sub_blue_;

//     ros::Publisher semantic_pub_red_;
//     ros::Publisher semantic_pub_blue_;

//     ros::Timer timer_;

//     void semanticCallback(const planner_semantic_msgs::SemanticPoint::ConstPtr& msg) {
//         geometry_msgs::Point point;
//         point.x = msg->point.x;
//         point.y = msg->point.y;
//         point.z = msg->point.z;
//         int description = msg->type.value;

//         points_.push_back({point, description});
//         ROS_INFO_STREAM("Collected point: (" << point.x << ", " << point.y << ", " << point.z << ") with description: " << description);
//     }

//     void odometryCallbackRed(const nav_msgs::Odometry::ConstPtr& msg) {
//         last_pose_red_ = msg->pose.pose.position;
//         // ROS_INFO_STREAM("Red robot current pose: (" << last_pose_red_.x << ", " << last_pose_red_.y << ", " << last_pose_red_.z << ")");
//         // ROS_INFO_STREAM("Calling checkProximityToFinalPose for red...");
//         checkProximityToFinalPose(last_pose_red_, target_pose_red_, reached_final_pose_red_, toggle_planning_client_red_, stop_planning_client_red_, automatic_planning_client_red_);
//     }

//     void odometryCallbackBlue(const nav_msgs::Odometry::ConstPtr& msg) {
//         last_pose_blue_ = msg->pose.pose.position;
//         // ROS_INFO_STREAM("Blue robot current pose: (" << last_pose_blue_.x << ", " << last_pose_blue_.y << ", " << last_pose_blue_.z << ")");
//         // ROS_INFO_STREAM("Calling checkProximityToFinalPose for blue...");
//         checkProximityToFinalPose(last_pose_blue_, target_pose_blue_, reached_final_pose_blue_, toggle_planning_client_blue_, stop_planning_client_blue_, automatic_planning_client_blue_);
//     }

//     void adjustedPointsCallbackRed(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
//         if (!msg->points.empty()) {
//             ROS_INFO_STREAM("trajec: (" << msg << ")");
//             target_pose_red_.x = msg->points.back().transforms[0].translation.x;
//             target_pose_red_.y = msg->points.back().transforms[0].translation.y;
//             target_pose_red_.z = msg->points.back().transforms[0].translation.z;
//             // ROS_INFO_STREAM("Set target pose for red: (" << target_pose_red_.x << ", " << target_pose_red_.y << ", " << target_pose_red_.z << ")");
//         }
//     }

//     void adjustedPointsCallbackBlue(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
//         if (!msg->points.empty()) {
//             target_pose_blue_.x = msg->points.back().transforms[0].translation.x;
//             target_pose_blue_.y = msg->points.back().transforms[0].translation.y;
//             target_pose_blue_.z = msg->points.back().transforms[0].translation.z;
//             // ROS_INFO_STREAM("Set target pose for blue: (" << target_pose_blue_.x << ", " << target_pose_blue_.y << ", " << target_pose_blue_.z << ")");
//         }
//     }

//     void checkProximityToFinalPose(const geometry_msgs::Point& current_pose, const geometry_msgs::Point& target_pose, bool& reached_final_pose, ros::ServiceClient& planning_client, ros::ServiceClient& stop_client, ros::ServiceClient& automatic_client) {
//         double dist = distance(current_pose, target_pose);
//         // ROS_INFO_STREAM("Checking distance to target pose: (" << target_pose.x << ", " << target_pose.y << ", " << target_pose.z << "), distance: " << dist);
//         if (dist < 0.5) {  // Check if within 1m³ vicinity
//             // ROS_INFO_STREAM("Robot reached the final pose vicinity: (" << target_pose.x << ", " << target_pose.y << ", " << target_pose.z << ")");
//             reached_final_pose = true;
//             reactivatePlannerIfNoTasks(planning_client, stop_client, automatic_client);
//         }
//     }

//     double distance(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2) {
//         return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
//     }

//     std::map<int, std::map<std::string, double>> calculateCosts() {
//         std::map<int, std::map<std::string, double>> costs;
//         for (size_t idx = 0; idx < points_.size(); ++idx) {
//             const auto& point = points_[idx].point;
//             planner_msgs::DistanceAndRole srv;
//             srv.request.point = point;

//             if (my_distance_service_red_.call(srv)) {
//                 costs[idx]["red"] = srv.response.distance;
//             } else {
//                 ROS_ERROR("Failed to call red distance service");
//             }

//             if (my_distance_service_blue_.call(srv)) {
//                 costs[idx]["blue"] = srv.response.distance;
//             } else {
//                 ROS_ERROR("Failed to call blue distance service");
//             }
//         }
//         return costs;
//     }

//     void filterTasksByProximity(std::vector<SemanticData>& tasks, double proximity_threshold) {
//         std::vector<SemanticData> filtered;
//         for (auto& task : tasks) {
//             bool is_too_close = std::any_of(filtered.begin(), filtered.end(), [&task, proximity_threshold, this](const SemanticData& other) {
//                 return distance(task.point, other.point) < proximity_threshold;
//             });
//             if (!is_too_close) {
//                 filtered.push_back(task);
//             }
//         }
//         tasks.swap(filtered);
//     }

//     void allocateTasks(const ros::TimerEvent&) {
//         if (points_.empty()) {
//             ROS_INFO("No points to allocate.");
//             return;
//         }

//         filterTasksByProximity(points_, 8.0);  // Example threshold of 8.0 units

//         // Stop the planners
//         std_srvs::SetBool toggle_srv;
//         toggle_srv.request.data = false;
//         if (toggle_planning_client_red_.call(toggle_srv) && toggle_planning_client_blue_.call(toggle_srv)) {
//             ROS_INFO("Planners stopped.");
//         } else {
//             ROS_ERROR("Failed to stop planners.");
//             return;
//         }

//         auto costs = calculateCosts();
//         red_tasks_.clear();
//         blue_tasks_.clear();

//         for (std::map<int, std::map<std::string, double>>::const_iterator it = costs.begin(); it != costs.end(); ++it) {
//             int idx = it->first;
//             const std::map<std::string, double>& cost = it->second;

//             if (cost.at("red") < cost.at("blue")) {
//                 red_tasks_.push_back(points_[idx]);
//             } else {
//                 blue_tasks_.push_back(points_[idx]);
//             }
//         }

//         // Log and publish tasks
//         logAndPublishTasks(red_tasks_, "red");
//         logAndPublishTasks(blue_tasks_, "blue");

//         // Print assigned tasks to ROS logs
//         ROS_INFO_STREAM("Assigned tasks to red robot:");
//         for (const auto& task : red_tasks_) {
//             ROS_INFO_STREAM("Task at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
//         }

//         ROS_INFO_STREAM("Assigned tasks to blue robot:");
//         for (const auto& task : blue_tasks_) {
//             ROS_INFO_STREAM("Task at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
//         }

//         points_.clear();  // Clear the list of points after allocation

//         // // Ensure tasks are cleared after processing
//         // red_tasks_.clear();
//         // blue_tasks_.clear();

//         // // Debugging after clearing
//         // ROS_INFO_STREAM("After clearing, red_tasks_ size: " << red_tasks_.size());
//         // ROS_INFO_STREAM("After clearing, blue_tasks_ size: " << blue_tasks_.size());
//     }

//     void logAndPublishTasks(const std::vector<SemanticData>& tasks, const std::string& robot_namespace) {
//         if (tasks.empty()) {
//             ROS_INFO_STREAM("No tasks assigned to " << robot_namespace << " robot. Enabling planner to continue exploring.");
//         } else {
//             ROS_INFO_STREAM("Assigned tasks to " << robot_namespace << " robot:");
//             for (const auto& task : tasks) {
//                 ROS_INFO_STREAM("Task at point: (" << task.point.x << ", " << task.point.y << ", " << task.point.z << ")");
//                 publishTask(task, robot_namespace);
//             }
//         }
//     }

//     void reactivatePlannerIfNoTasks(ros::ServiceClient& planning_client, ros::ServiceClient& stop_client, ros::ServiceClient& automatic_client) {
//         std_srvs::SetBool toggle_srv;
//         toggle_srv.request.data = true; // Enable planner
//         if (planning_client.call(toggle_srv)) {
//             ROS_INFO("Planner restarted for exploration.");

//             // Trigger the stop and automatic planning services
//             std_srvs::Trigger trigger_srv;
            
//             if (stop_client.call(trigger_srv) && trigger_srv.response.success) {
//                 ROS_INFO("Stop service called successfully.");
//             } else {
//                 ROS_WARN("Failed to call stop service.");
//             }

//             if (automatic_client.call(trigger_srv) && trigger_srv.response.success) {
//                 ROS_INFO("Automatic planning service called successfully.");
//             } else {
//                 ROS_WARN("Failed to call automatic planning service.");
//             }
//         } else {
//             ROS_ERROR("Failed to restart planner.");
//         }
//     }

//     void publishTask(const SemanticData& task, const std::string& robot_namespace) {
//         planner_semantic_msgs::SemanticPoint msg;
//         msg.header.stamp = ros::Time::now();
//         msg.header.frame_id = "world";
//         msg.type.value = task.description;
//         msg.point.x = task.point.x;
//         msg.point.y = task.point.y;
//         msg.point.z = task.point.z;

//         if (robot_namespace == "red") {
//             semantic_pub_red_.publish(msg);
//             ROS_INFO_STREAM("Published task for red robot at point: (" << msg.point.x << ", " << msg.point.y << ", " << msg.point.z << ")");
//         } else {
//             semantic_pub_blue_.publish(msg);
//             ROS_INFO_STREAM("Published task for blue robot at point: (" << msg.point.x << ", " << msg.point.y << ", " << msg.point.z << ")");
//         }
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "dynamic_auction");
//     DynamicTaskAllocator allocator;
//     ros::spin();
//     return 0;
// }
