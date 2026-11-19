// #include <ros/ros.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <tf/tf.h>

// class TrajectoryTransformer {
// public:
//     TrajectoryTransformer(ros::NodeHandle& nh, const std::string& sub_topic, const std::string& pub_topic, double offset_x, double offset_y)
//     : nh_(nh), sub_topic_(sub_topic), pub_topic_(pub_topic), offset_x_(offset_x), offset_y_(offset_y) {

//         // Subscribe to the input topic and prepare to publish to the output topic
//         sub_ = nh_.subscribe(sub_topic_, 10, &TrajectoryTransformer::trajectoryCallback, this);
//         pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(pub_topic_, 10);
//     }

//     void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
//         // ROS_INFO("Redirecting trajectory from %s to %s", sub_topic_.c_str(), pub_topic_.c_str());
//         trajectory_msgs::MultiDOFJointTrajectory transformed_trajectory = *msg; // Clone the message

//         // Apply transformation to each point in the trajectory
//         for (auto& point : transformed_trajectory.points) {
//             for (auto& transform : point.transforms) {
//                 // Translate coordinates by subtracting the offset
//                 double world_x = transform.translation.x - offset_x_;  // Subtract robot x-offset
//                 double world_y = transform.translation.y - offset_y_;  // Subtract robot y-offset
//                 // Rotate 90 degrees CCW around the z-axis
//                 transform.translation.x = -world_y;  // New x is negative old y
//                 transform.translation.y = world_x;   // New y is old x
//             }
//         }

//         // Publish the transformed trajectory
//         pub_.publish(transformed_trajectory);
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber sub_;
//     ros::Publisher pub_;
//     std::string sub_topic_;
//     std::string pub_topic_;
//     double offset_x_, offset_y_;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "trajectory_transformer");
//     ros::NodeHandle nh;

//     // Setup transformation with specific offsets and topics
//      //for one 
//     // TrajectoryTransformer red_to_red_1(nh, "/red/command/trajectory", "/red/tracker/input_trajectory", -10, -10);
//     // TrajectoryTransformer red_to_red_2(nh, "/red/inspection_trajectory", "/red/tracker/input_trajectory", -10, -10);
//     // TrajectoryTransformer red_to_blue_3(nh, "/red/near_obstacle", "/blue/tracker/input_trajectory", -10, -10);

//     //for two
//     TrajectoryTransformer red_to_blue_1(nh, "/red/command/trajectory", "/blue/tracker/input_trajectory", -10, -10);
//     TrajectoryTransformer blue_to_red_1(nh, "/blue/command/trajectory", "/red/tracker/input_trajectory", -20, -10);
//     TrajectoryTransformer red_to_blue_2(nh, "/red/inspection_trajectory", "/blue/tracker/input_trajectory", -10, -10);
//     TrajectoryTransformer blue_to_red_2(nh, "/blue/inspection_trajectory", "/red/tracker/input_trajectory", -20, -10);

//     TrajectoryTransformer red_to_blue_3(nh, "/red/near_obstacle", "/blue/tracker/input_trajectory", -10, -10);
//     TrajectoryTransformer blue_to_red_3(nh, "/blue/near_obstacle", "/red/tracker/input_trajectory", -20, -10);
//     ros::spin();
//     return 0;
// }


//////////////////////////////////------------------------------------for single with marker 

// #include <ros/ros.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>
// #include <visualization_msgs/Marker.h>
// #include <tf/tf.h>

// class TrajectoryTransformer {
// public:
//     TrajectoryTransformer(const std::string& sub_topic, const std::string& pub_topic, double offset_x, double offset_y, const std::string& ns)
//     : sub_topic_(sub_topic), pub_topic_(pub_topic), offset_x_(offset_x), offset_y_(offset_y), ns_(ns) {}

//     void initialize(ros::NodeHandle& nh) {
//         sub_ = nh.subscribe(sub_topic_, 10, &TrajectoryTransformer::trajectoryCallback, this);
//         pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(pub_topic_, 10);
//         marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_path_" + ns_, 10);
//     }

//     void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
//         // Publish the original trajectory as a marker before transformation
//         publishMarker(msg, "original");

//         trajectory_msgs::MultiDOFJointTrajectory transformed_trajectory = *msg; // Clone the message

//         // Apply transformation to each point in the trajectory
//         for (auto& point : transformed_trajectory.points) {
//             for (auto& transform : point.transforms) {
//                 // Translate coordinates by subtracting the offset
//                 double world_x = transform.translation.x - offset_x_;  // Subtract robot x-offset
//                 double world_y = transform.translation.y - offset_y_;  // Subtract robot y-offset
//                 // Rotate 90 degrees CCW around the z-axis
//                 transform.translation.x = -world_y;  // New x is negative old y
//                 transform.translation.y = world_x;   // New y is old x
//             }
//         }

//         // Publish the transformed trajectory
//         pub_.publish(transformed_trajectory);

//         // Publish the transformed trajectory as a marker
//         publishMarker(transformed_trajectory, "transformed");
//     }

// private:
//     void publishMarker(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg, const std::string& marker_ns) {
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "world";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = marker_ns;
//         marker.id = 0;
//         marker.type = visualization_msgs::Marker::LINE_STRIP;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.scale.x = 0.1; // Line width
//         marker.color.a = 1.0;
//         if (marker_ns == "original") {
//             marker.color.r = 0.0;
//             marker.color.g = 1.0;
//             marker.color.b = 0.0;
//         } else {
//             marker.color.r = 1.0;
//             marker.color.g = 0.0;
//             marker.color.b = 0.0;
//         }

//         for (const auto& point : msg->points) {
//             for (const auto& transform : point.transforms) {
//                 geometry_msgs::Point p;
//                 p.x = transform.translation.x;
//                 p.y = transform.translation.y;
//                 p.z = transform.translation.z;
//                 marker.points.push_back(p);
//             }
//         }

//         marker_pub_.publish(marker);
//     }

//     void publishMarker(const trajectory_msgs::MultiDOFJointTrajectory& msg, const std::string& marker_ns) {
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "world";
//         marker.header.stamp = ros::Time::now();
//         marker.ns = marker_ns;
//         marker.id = 0;
//         marker.type = visualization_msgs::Marker::LINE_STRIP;
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.scale.x = 0.1; // Line width
//         marker.color.a = 1.0;
//         if (marker_ns == "original") {
//             marker.color.r = 0.0;
//             marker.color.g = 1.0;
//             marker.color.b = 0.0;
//         } else {
//             marker.color.r = 1.0;
//             marker.color.g = 0.0;
//             marker.color.b = 0.0;
//         }

//         for (const auto& point : msg.points) {
//             for (const auto& transform : point.transforms) {
//                 geometry_msgs::Point p;
//                 p.x = transform.translation.x;
//                 p.y = transform.translation.y;
//                 p.z = transform.translation.z;
//                 marker.points.push_back(p);
//             }
//         }

//         marker_pub_.publish(marker);
//     }

//     std::string sub_topic_;
//     std::string pub_topic_;
//     double offset_x_, offset_y_;
//     std::string ns_;
//     ros::Subscriber sub_;
//     ros::Publisher pub_;
//     ros::Publisher marker_pub_; // Publisher for visualization markers
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "trajectory_transformer");
//     ros::NodeHandle nh;

//     // Setup transformation with specific offsets and topics
//     TrajectoryTransformer red_to_red_1("/red/command/trajectory", "/red/tracker/input_trajectory", -10, -10, "red_to_red_1");
//     TrajectoryTransformer red_to_red_2("/red/inspection_trajectory", "/red/tracker/input_trajectory", -10, -10, "red_to_red_2");
//     TrajectoryTransformer red_to_red_3("/red/near_obstacle", "/red/tracker/input_trajectory", -10, -10, "red_to_red_3");

//     // Initialize the subscribers and publishers
//     red_to_red_1.initialize(nh);
//     red_to_red_2.initialize(nh);
//     red_to_red_3.initialize(nh);

//     // Uncomment these lines for additional transformations
//     // TrajectoryTransformer red_to_blue_1("/red/command/trajectory", "/blue/tracker/input_trajectory", -10, -10, "red_to_blue_1");
//     // TrajectoryTransformer blue_to_red_1("/blue/command/trajectory", "/red/tracker/input_trajectory", -20, -10, "blue_to_red_1");
//     // TrajectoryTransformer red_to_blue_2("/red/inspection_trajectory", "/blue/tracker/input_trajectory", -10, -10, "red_to_blue_2");
//     // TrajectoryTransformer blue_to_red_2("/blue/inspection_trajectory", "/red/tracker/input_trajectory", -20, -10, "blue_to_red_2");

//     // red_to_blue_1.initialize(nh);
//     // blue_to_red_1.initialize(nh);
//     // red_to_blue_2.initialize(nh);
//     // blue_to_red_2.initialize(nh);

//     // Uncomment these lines for near obstacle transformations
//     // TrajectoryTransformer red_to_blue_3("/red/near_obstacle", "/blue/tracker/input_trajectory", -10, -10, "red_to_blue_3");
//     // TrajectoryTransformer blue_to_red_3("/blue/near_obstacle", "/red/tracker/input_trajectory", -20, -10, "blue_to_red_3");

//     // red_to_blue_3.initialize(nh);
//     // blue_to_red_3.initialize(nh);

//     ros::spin();
//     return 0;
// }

///////////////////////////////////________for double with marker

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

class TrajectoryTransformer {
public:
    TrajectoryTransformer(ros::NodeHandle& nh, const std::string& sub_topic, const std::string& pub_topic, double offset_x, double offset_y, const std::string& ns)
    : nh_(nh), sub_topic_(sub_topic), pub_topic_(pub_topic), offset_x_(offset_x), offset_y_(offset_y), ns_(ns) {

        // Subscribe to the input topic and prepare to publish to the output topic
        sub_ = nh_.subscribe(sub_topic_, 10, &TrajectoryTransformer::trajectoryCallback, this);
        pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(pub_topic_, 10);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_path_" + ns_, 10);
    }

    void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
        trajectory_msgs::MultiDOFJointTrajectory transformed_trajectory = *msg; // Clone the message

                // Create a marker for visualization
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns_;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1; // Line width
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Apply transformation to each point in the trajectory
        for (auto& point : transformed_trajectory.points) {
            for (auto& transform : point.transforms) {
                // Translate coordinates by subtracting the offset
                double world_x = transform.translation.x - offset_x_;  // Subtract robot x-offset
                double world_y = transform.translation.y - offset_y_;  // Subtract robot y-offset
                // Rotate 90 degrees CCW around the z-axis
                transform.translation.x = -world_y;  // New x is negative old y
                transform.translation.y = world_x;   // New y is old x

                // Add the transformed point to the marker
                geometry_msgs::Point p;
                p.x = transform.translation.x;
                p.y = transform.translation.y;
                p.z = transform.translation.z;
                marker.points.push_back(p);
            }
        }

        // Publish the transformed trajectory
        pub_.publish(transformed_trajectory);

        // Publish the marker
        marker_pub_.publish(marker);
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher marker_pub_; // Publisher for visualization markers
    std::string sub_topic_;
    std::string pub_topic_;
    double offset_x_, offset_y_;
    std::string ns_;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_transformer");
    ros::NodeHandle nh("~");  // Private NodeHandle to access node-specific parameters

    // Variables for parameters
    double offset_x_red, offset_y_red, offset_x_blue, offset_y_blue;

    // Check if parameters exist and read them
    if (nh.hasParam("offset_x_red")) {
        nh.getParam("offset_x_red", offset_x_red);
        ROS_INFO("Parameter offset_x_red found: %f", offset_x_red);
    } else {
        offset_x_red = -13.0; // Default value
        ROS_WARN("Parameter offset_x_red not found, using default: %f", offset_x_red);
    }

    if (nh.hasParam("offset_y_red")) {
        nh.getParam("offset_y_red", offset_y_red);
        ROS_INFO("Parameter offset_y_red found: %f", offset_y_red);
    } else {
        offset_y_red = 10.0; // Default value
        ROS_WARN("Parameter offset_y_red not found, using default: %f", offset_y_red);
    }

    if (nh.hasParam("offset_x_blue")) {
        nh.getParam("offset_x_blue", offset_x_blue);
        ROS_INFO("Parameter offset_x_blue found: %f", offset_x_blue);
    } else {
        offset_x_blue = -25.0; // Default value
        ROS_WARN("Parameter offset_x_blue not found, using default: %f", offset_x_blue);
    }

    if (nh.hasParam("offset_y_blue")) {
        nh.getParam("offset_y_blue", offset_y_blue);
        ROS_INFO("Parameter offset_y_blue found: %f", offset_y_blue);
    } else {
        offset_y_blue = 10.0; // Default value
        ROS_WARN("Parameter offset_y_blue not found, using default: %f", offset_y_blue);
    }

    // Initialize TrajectoryTransformer instances using the parameters
    TrajectoryTransformer red_to_blue_1(nh, "/red/command/trajectory", "/blue/tracker/input_trajectory", offset_x_red, offset_y_red, "red_to_blue_1");
    TrajectoryTransformer blue_to_red_1(nh, "/blue/command/trajectory", "/red/tracker/input_trajectory", offset_x_blue, offset_y_blue, "blue_to_red_1");
    TrajectoryTransformer red_to_blue_2(nh, "/red/inspection_trajectory", "/blue/tracker/input_trajectory", offset_x_red, offset_y_red, "red_to_blue_2");
    TrajectoryTransformer blue_to_red_2(nh, "/blue/inspection_trajectory", "/red/tracker/input_trajectory", offset_x_blue, offset_y_blue, "blue_to_red_2");
    TrajectoryTransformer red_to_blue_3(nh, "/red/near_obstacle", "/blue/tracker/input_trajectory", offset_x_red, offset_y_red, "red_to_blue_3");
    TrajectoryTransformer blue_to_red_3(nh, "/blue/near_obstacle", "/red/tracker/input_trajectory", offset_x_blue, offset_y_blue, "blue_to_red_3");

    ros::spin();
    return 0;
}