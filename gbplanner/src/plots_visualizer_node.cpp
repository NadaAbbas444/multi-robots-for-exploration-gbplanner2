// #include <ros/ros.h>
// #include <voxblox_msgs/Layer.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <voxblox/core/layer.h>
// #include <voxblox/core/block.h>
// #include <voxblox/core/tsdf_map.h>
// #include <voxblox/core/voxel.h>
// #include <voxblox/integrator/tsdf_integrator.h>
// #include <voxblox/utils/layer_utils.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <std_msgs/String.h>
// #include <sstream>
// #include <fstream>

// enum VoxelType { OCCUPIED, FREE, UNMAPPED };

// VoxelType classifyVoxel(const voxblox::TsdfVoxel& voxel) {
//     if (voxel.distance < 0.0f && voxel.weight > 0.0f) {
//         return OCCUPIED;
//     } else if (voxel.distance > 0.0f && voxel.weight > 0.0f) {
//         return FREE;
//     } else {
//         return UNMAPPED;
//     }
// }

// void processSurfacePointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg, const voxblox::Point& center_point, double radius) {
//     pcl::PointCloud<pcl::PointXYZ> cloud;
//     pcl::fromROSMsg(*msg, cloud);

//     double surface_volume = 0.0;
//     double voxel_volume = std::pow(0.2, 3);  // Assuming voxel size is 0.2, adjust as needed

//     std::stringstream ss;
//     ss << "Processing Surface Point Cloud\n";

//     int points_within_radius = 0;
//     for (const auto& point : cloud.points) {
//         if ((voxblox::Point(point.x, point.y, point.z) - center_point).norm() <= radius) {
//             surface_volume += voxel_volume;
//             ++points_within_radius;
//         }
//     }

//     ss << "Surface volume: " << surface_volume << " cubic meters\n";

//     // Save to file
//     std::ofstream outfile("/home/nadaabbas/Downloads/thesis/results/plots/inspection_v3t/graph_based_inspection_truck.txt", std::ios_base::app);
//     outfile << ros::Time::now().toSec() << " 0 0 0 " << surface_volume << "\n";
//     outfile.close();

//     ROS_INFO_STREAM(ss.str());
// }

// void tsdfCallback(const voxblox_msgs::Layer::ConstPtr& msg, ros::Publisher& pub, const voxblox::Point& center_point, double radius) {
//     ROS_INFO("tsdfCallback called.");

//     double voxel_size = msg->voxel_size;
//     double voxel_volume = voxel_size * voxel_size * voxel_size;
//     double occupied_volume = 0.0;
//     double free_volume = 0.0;
//     double unmapped_volume = 0.0;

//     std::stringstream ss;
//     ss << "Received TSDF Layer message\n";

//     int voxels_processed = 0;
//     for (const auto& block_msg : msg->blocks) {
//         voxblox::Point block_origin(
//             static_cast<double>(block_msg.x_index) * msg->voxels_per_side * voxel_size,
//             static_cast<double>(block_msg.y_index) * msg->voxels_per_side * voxel_size,
//             static_cast<double>(block_msg.z_index) * msg->voxels_per_side * voxel_size);

//         if ((block_origin - center_point).cwiseAbs().maxCoeff() > radius + msg->voxels_per_side * voxel_size) {
//             continue; // Skip blocks that are completely outside the radius
//         }

//         voxblox::Block<voxblox::TsdfVoxel> block(msg->voxels_per_side, voxel_size, block_origin);

//         size_t voxel_data_size = sizeof(voxblox::TsdfVoxel);
//         for (size_t i = 0; i < block_msg.data.size() / voxel_data_size; ++i) {
//             voxblox::TsdfVoxel voxel;
//             std::memcpy(&voxel, &block_msg.data[i * voxel_data_size], voxel_data_size);

//             voxblox::Point voxel_coords = block.computeCoordinatesFromLinearIndex(i);
//             if ((voxel_coords - center_point).norm() > radius) {
//                 continue; // Skip voxels outside the radius
//             }

//             VoxelType voxel_type = classifyVoxel(voxel);
//             switch (voxel_type) {
//                 case OCCUPIED:
//                     occupied_volume += voxel_volume;
//                     break;
//                 case FREE:
//                     free_volume += voxel_volume;
//                     break;
//                 case UNMAPPED:
//                     unmapped_volume += voxel_volume;
//                     break;
//             }
//             ++voxels_processed;
//         }
//     }

//     ss << "Occupied volume: " << occupied_volume << " cubic meters\n";
//     ss << "Free volume: " << free_volume << " cubic meters\n";
//     ss << "Unmapped volume: " << unmapped_volume << " cubic meters\n";

//     std_msgs::String info_msg;
//     info_msg.data = ss.str();
//     pub.publish(info_msg);

//     ROS_INFO_STREAM(info_msg.data);

//     // Save to file
//     std::ofstream outfile("/home/nadaabbas/Downloads/thesis/results/plots/inspection_v3t/graph_based_inspection_truck.txt", std::ios_base::app);
//     double current_time = ros::Time::now().toSec();
//     outfile << current_time << " " << occupied_volume << " " << free_volume << " " << unmapped_volume << "\n";
//     outfile.close();
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "plots_visualizer_node");
//     ros::NodeHandle nh;

//     // Define the center point and radius
//     voxblox::Point center_point(-1, -7.15, 3.0); // Set the desired center point here
//     double radius = 4.0; 

//     ros::Publisher info_pub = nh.advertise<std_msgs::String>("tsdf_info", 10);

//     ros::Subscriber tsdf_sub = nh.subscribe<voxblox_msgs::Layer>("/red/gbplanner_node/tsdf_map_out", 10,
//         boost::bind(tsdfCallback, _1, boost::ref(info_pub), boost::ref(center_point), radius));

//     ros::Subscriber surface_sub = nh.subscribe<sensor_msgs::PointCloud2>("/red/gbplanner_node/surface_pointcloud", 10,
//         boost::bind(processSurfacePointCloud, _1, boost::ref(center_point), radius));

//     ROS_INFO("tsdf_subscriber_publisher_node started, listening on /gbplanner_node/tsdf_map_out and /red/gbplanner_node/surface_pointcloud");

//     ros::spin();
//     return 0;
// }


//__________________________________________________________


// #include <ros/ros.h>
// #include <voxblox_msgs/Mesh.h>
// #include <voxblox_msgs/MeshBlock.h>
// #include <Eigen/Dense>
// #include <vector>

// ros::Publisher mesh_pub;

// void meshCallback(const voxblox_msgs::Mesh::ConstPtr& msg) {
//     voxblox_msgs::Mesh cropped_mesh;
//     cropped_mesh.header = msg->header;
//     cropped_mesh.block_edge_length = msg->block_edge_length;

//     // Define the cropping boxes
//     Eigen::Vector3f center1(-22.0, 5.14, 3.0);
//     Eigen::Vector3f center2(0.9, -7.0, 3.0);
//     float box_size = 5.5;  // Half of 10x10x10

//     Eigen::Vector3f min_point1 = center1 - Eigen::Vector3f(box_size, box_size, box_size);
//     Eigen::Vector3f max_point1 = center1 + Eigen::Vector3f(box_size, box_size, box_size);
//     Eigen::Vector3f min_point2 = center2 - Eigen::Vector3f(box_size, box_size, box_size);
//     Eigen::Vector3f max_point2 = center2 + Eigen::Vector3f(box_size, box_size, box_size);

//     size_t total_vertices = 0;

//     for (const auto& block : msg->mesh_blocks) {
//         voxblox_msgs::MeshBlock cropped_block;
//         cropped_block.index = block.index;
//         std::vector<uint16_t> new_x, new_y, new_z;
//         std::vector<uint8_t> new_r, new_g, new_b;
//         size_t vertex_count = block.x.size();

//         if (vertex_count == block.y.size() && vertex_count == block.z.size()) {
//             Eigen::Vector3f block_origin(
//                 block.index[0] * msg->block_edge_length,
//                 block.index[1] * msg->block_edge_length,
//                 block.index[2] * msg->block_edge_length
//             );

//             for (size_t i = 0; i < vertex_count; ++i) {
//                 float x = block_origin.x() + block.x[i] * msg->block_edge_length / 65535.0f;
//                 float y = block_origin.y() + block.y[i] * msg->block_edge_length / 65535.0f;
//                 float z = block_origin.z() + block.z[i] * msg->block_edge_length / 65535.0f;

//                 bool inside_box1 = x >= min_point1.x() && x <= max_point1.x() &&
//                                    y >= min_point1.y() && y <= max_point1.y() &&
//                                    z >= min_point1.z() && z <= max_point1.z();
                
//                 bool inside_box2 = x >= min_point2.x() && x <= max_point2.x() &&
//                                    y >= min_point2.y() && y <= max_point2.y() &&
//                                    z >= min_point2.z() && z <= max_point2.z();

//                 if (inside_box1 || inside_box2) {
//                     new_x.push_back(block.x[i]);
//                     new_y.push_back(block.y[i]);
//                     new_z.push_back(block.z[i]);

//                     if (block.r.size() == vertex_count && block.g.size() == vertex_count && block.b.size() == vertex_count) {
//                         new_r.push_back(block.r[i]);
//                         new_g.push_back(block.g[i]);
//                         new_b.push_back(block.b[i]);
//                     }
//                 }
//             }

//             // Ensure that the vertices are correctly grouped into triangles
//             if (new_x.size() % 3 == 0) {
//                 cropped_block.x = new_x;
//                 cropped_block.y = new_y;
//                 cropped_block.z = new_z;
//                 cropped_block.r = new_r;
//                 cropped_block.g = new_g;
//                 cropped_block.b = new_b;
//                 cropped_mesh.mesh_blocks.push_back(cropped_block);
//                 total_vertices += new_x.size();
//             } else {
//                 ROS_WARN("Vertices count is not a multiple of 3, skipping this block.");
//             }
//         } else {
//             ROS_WARN("Mismatch in block sizes. Skipping this block.");
//         }
//     }

//     mesh_pub.publish(cropped_mesh);
//     ROS_INFO("Cropped mesh published with %zu vertices and %zu blocks.", total_vertices, cropped_mesh.mesh_blocks.size());
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "plots_visualizer_node");
//     ros::NodeHandle nh;

//     ros::Subscriber mesh_sub = nh.subscribe<voxblox_msgs::Mesh>("/red/gbplanner_node/mesh", 10, meshCallback);
//     mesh_pub = nh.advertise<voxblox_msgs::Mesh>("/red/gbplanner_node/cropped_mesh", 10);

//     ROS_INFO("mesh_cropper_node started, listening on /red/gbplanner_node/mesh");

//     ros::spin();
//     return 0;
// }

//----------------------------------


// #include <ros/ros.h>
// #include <voxblox_msgs/Mesh.h>
// #include <voxblox_msgs/MeshBlock.h>
// #include <pcl/PolygonMesh.h>
// #include <pcl/io/vtk_lib_io.h>
// #include <Eigen/Dense>
// #include <vector>

// void meshCallback(const voxblox_msgs::Mesh::ConstPtr& msg) {
//     pcl::PolygonMesh polygon_mesh;
//     pcl::PointCloud<pcl::PointXYZRGB> cloud;
//     std::vector<pcl::Vertices> polygons;

//     // Define the cropping box centered around (0, -7, 2) with size 5x5x5
//     Eigen::Vector3f min_point(-2.5, -9.5, -0.5);
//     Eigen::Vector3f max_point(2.5, -4.5, 4.5);

//     for (const auto& block : msg->mesh_blocks) {
//         size_t vertex_count = block.x.size();

//         if (vertex_count == block.y.size() && vertex_count == block.z.size()) {
//             Eigen::Vector3f block_origin(
//                 block.index[0] * msg->block_edge_length,
//                 block.index[1] * msg->block_edge_length,
//                 block.index[2] * msg->block_edge_length
//             );

//             for (size_t i = 0; i < vertex_count; ++i) {
//                 float x = block_origin.x() + block.x[i] * msg->block_edge_length / 65535.0f;
//                 float y = block_origin.y() + block.y[i] * msg->block_edge_length / 65535.0f;
//                 float z = block_origin.z() + block.z[i] * msg->block_edge_length / 65535.0f;

//                 if (x >= min_point.x() && x <= max_point.x() &&
//                     y >= min_point.y() && y <= max_point.y() &&
//                     z >= min_point.z() && z <= max_point.z()) {
//                     pcl::PointXYZRGB point;
//                     point.x = x;
//                     point.y = y;
//                     point.z = z;
//                     point.r = (block.r.size() == vertex_count) ? block.r[i] : 255;
//                     point.g = (block.g.size() == vertex_count) ? block.g[i] : 255;
//                     point.b = (block.b.size() == vertex_count) ? block.b[i] : 255;
//                     cloud.push_back(point);
//                 }
//             }

//             // Construct triangles
//             size_t num_triangles = vertex_count / 3;
//             for (size_t i = 0; i < num_triangles; ++i) {
//                 pcl::Vertices triangle;
//                 triangle.vertices.push_back(i * 3);
//                 triangle.vertices.push_back(i * 3 + 1);
//                 triangle.vertices.push_back(i * 3 + 2);
//                 polygons.push_back(triangle);
//             }
//         } else {
//             ROS_WARN("Mismatch in block sizes. Skipping this block.");
//         }
//     }

//     // Convert the point cloud to a PCL PolygonMesh
//     pcl::toPCLPointCloud2(cloud, polygon_mesh.cloud);
//     polygon_mesh.polygons = polygons;

//     // Save the polygon mesh to a PLY file
//     std::string filename = "/home/nadaabbas/Downloads/thesis/results/cropped_mesh.ply";
//     pcl::io::savePolygonFilePLY(filename, polygon_mesh);
//     ROS_INFO("Cropped mesh saved to %s with %zu vertices and %zu polygons.", filename.c_str(), cloud.size(), polygons.size());
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "mesh_saver_node");
//     ros::NodeHandle nh;

//     ros::Subscriber mesh_sub = nh.subscribe<voxblox_msgs::Mesh>("/red/gbplanner_node/mesh", 10, meshCallback);

//     ROS_INFO("mesh_saver_node started, listening on /red/gbplanner_node/mesh");

//     ros::spin();
//     return 0;
// }


// ____________________________________ task allocation vis_____________________________________
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Transform.h>
#include <fstream>
#include <string>
#include <cmath>

class PlotsVisualizerNode {
public:
    PlotsVisualizerNode() {
        red_sub_ = nh_.subscribe("/red/near_obstacle", 10, &PlotsVisualizerNode::redCallback, this);
        blue_sub_ = nh_.subscribe("/blue/near_obstacle", 10, &PlotsVisualizerNode::blueCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber red_sub_;
    ros::Subscriber blue_sub_;
    std::string file_path_ = "/home/nadaabbas/Downloads/thesis/results/plots/ta/auction_tunnel_env.txt";

    void redCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
        processTrajectory("red", msg);
    }

    void blueCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
        processTrajectory("blue", msg);
    }

    void processTrajectory(const std::string& namespace_name, const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
        double total_distance = 0.0;
        std::vector<geometry_msgs::Transform> transforms;

        for (size_t i = 0; i < msg->points.size(); ++i) {
            const auto& point = msg->points[i];
            const auto& transform = point.transforms[0];
            transforms.push_back(transform);

            if (i > 0) {
                const auto& prev_transform = msg->points[i - 1].transforms[0];
                double dx = transform.translation.x - prev_transform.translation.x;
                double dy = transform.translation.y - prev_transform.translation.y;
                double dz = transform.translation.z - prev_transform.translation.z;
                total_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
            }
        }

        saveToFile(namespace_name, total_distance, transforms);
    }

    void saveToFile(const std::string& namespace_name, double total_distance, const std::vector<geometry_msgs::Transform>& transforms) {
        std::ofstream outfile(file_path_, std::ios_base::app);
        if (!outfile.is_open()) {
            ROS_ERROR("Failed to open file: %s", file_path_.c_str());
            return;
        }

        outfile << namespace_name << " " << total_distance << " ";
        for (const auto& transform : transforms) {
            outfile << "(" << transform.translation.x << "," << transform.translation.y << "," << transform.translation.z << ") ";
        }
        outfile << "\n";
        outfile.close();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "plots_visualizer_node");
    PlotsVisualizerNode node;
    ros::spin();
    return 0;
}
