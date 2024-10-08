#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zivid_interfaces/srv/capture_assistant_suggest_settings.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <tuple>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*
 * This node uses the Zivid ROS capture assistant to capture point clouds,
 * detects red cylinder objects, and saves the x, y, z coordinates of each detected object.
 */
class ObjectDetection : public rclcpp::Node
{
public:
    ObjectDetection() : Node("object_detection")
    {
        // Set up transform listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Set up capture service client
        capture_client_ = this->create_client<std_srvs::srv::Trigger>("capture");
        while (!capture_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for capture service...");
        }

        // Set up Capture Assistant Suggest Settings service client
        capture_assistant_client_ = this->create_client<zivid_interfaces::srv::CaptureAssistantSuggestSettings>("capture_assistant/suggest_settings");
        while (!capture_assistant_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for capture assistant service...");
        }

        // Subscribe to the point cloud topic from the Zivid camera
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points/xyzrgba", 10, std::bind(&ObjectDetection::pointCloudCallback, this, std::placeholders::_1));

        // Set capture assistant settings and trigger the first capture
        suggestCaptureSettingsAndCapture();
    }

private:
    void suggestCaptureSettingsAndCapture()
    {
        RCLCPP_INFO(this->get_logger(), "Requesting Capture Assistant for optimal settings...");

        auto request = std::make_shared<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request>();
        request->max_capture_time = rclcpp::Duration::from_seconds(10.0);  // No concern for time
        request->ambient_light_frequency = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_50HZ;

        auto result = capture_assistant_client_->async_send_request(request);

        // Wait for the result of the service call
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call capture assistant");
        } else {
            RCLCPP_INFO(this->get_logger(), "Capture Assistant has suggested settings, now capturing...");
            triggerCapture();
        }
    }

    void triggerCapture()
    {
        // Create a request to trigger the capture service
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = capture_client_->async_send_request(request);

        // Wait for the result of the service call
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call capture service");
        } else {
            RCLCPP_INFO(this->get_logger(), "Capture service called successfully");
        }
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Point cloud received");

        // Convert the received ROS PointCloud2 message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Vector to store red points
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr red_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

        // Vector to store the positions of detected red cylinders
        //std::vector<std::tuple<float, float, float>> red_cylinder_positions;

        // Iterate through points to detect red color (adjust color thresholds as needed)
        for (const auto& point : cloud.points) {
            // Check if the point has valid (non-NaN) coordinates
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
                // Simple red detection
                if (point.r > 65 && point.g < 10 && point.b < 10) {
                    red_cloud->points.push_back(point);
                }
            }
        }

        // Save detected red cylinder positions
        if (red_cloud->points.empty()) {
            RCLCPP_INFO(this->get_logger(), "No red cylinders detected.");
            return;
        }

        // Step 1: Perform Euclidean Cluster Extraction to group points that belong to the same object
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(red_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        ec.setClusterTolerance(0.005);  // Distance between points in a cluster
        ec.setMinClusterSize(10);      // Minimum number of points that form a cluster
        ec.setMaxClusterSize(10000);   // Maximum number of points in a cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(red_cloud);
        ec.extract(cluster_indices);
        
        RCLCPP_INFO(this->get_logger(), "Number of red cylinders detected: %lu", cluster_indices.size());

        // Step 2: Calculate centroids of each cluster
        std::vector<std::tuple<float, float, float>> centroids;
        std::vector<Eigen::Vector3d> camera_points;  // Points in the camera frame
        std::vector<Eigen::Vector3d> real_world_points;  // Points in the real world frame (entered by user)
        std::vector<std::tuple<double, double, double>> real_world_centroids_;  // To store real-world measurements

        for (const auto& cluster : cluster_indices) {
            float x_sum = 0, y_sum = 0, z_sum = 0;
            int num_points = cluster.indices.size();

            for (const auto& idx : cluster.indices) {
                const auto& point = red_cloud->points[idx];
                x_sum += point.x;
                y_sum += point.y;
                z_sum += point.z;
            }

            float x_centroid = x_sum / num_points;
            float y_centroid = y_sum / num_points;
            float z_centroid = z_sum / num_points;

            centroids.push_back(std::make_tuple(x_centroid, y_centroid, z_centroid));
        }

        // Step 3: Transform centroids from camera frame to world frame
        std::string target_frame = "world";  // Replace this with your world frame
        std::string source_frame = "zivid_optical_frame";  // Frame of the Zivid camera

        for (size_t i = 0; i < centroids.size(); ++i) {
            geometry_msgs::msg::PointStamped point_camera_frame;
            point_camera_frame.header.frame_id = source_frame;
            point_camera_frame.point.x = std::get<0>(centroids[i]);
            point_camera_frame.point.y = std::get<1>(centroids[i]);
            point_camera_frame.point.z = std::get<2>(centroids[i]);

            try {
                auto transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                geometry_msgs::msg::PointStamped point_world_frame;
                tf2::doTransform(point_camera_frame, point_world_frame, transform);

                RCLCPP_INFO(this->get_logger(), "Red cylinder centroid in camera frame: x: %.3f, y: %.3f, z: %.3f",
                            point_camera_frame.point.x, point_camera_frame.point.y, point_camera_frame.point.z);
                RCLCPP_INFO(this->get_logger(), "Red cylinder centroid in world frame: x: %.3f, y: %.3f, z: %.3f",
                            point_world_frame.point.x, point_world_frame.point.y, point_world_frame.point.z);
                // Store the detected camera frame points
                camera_points.emplace_back(point_world_frame.point.x, point_world_frame.point.y, point_world_frame.point.z);

            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            }
            
            // Prompt the user to input real-world coordinates
            double x_real, y_real, z_real;
            std::cout << "Enter real-world coordinates for object " << i + 1 << " (x, y, z): ";
            std::cin >> x_real >> y_real >> z_real;

            // Store the real-world coordinates entered by the user
            real_world_points.emplace_back(x_real, y_real, z_real);

            RCLCPP_INFO(this->get_logger(), "Real-world measurement entered for object %lu: x: %.3f, y: %.3f, z: %.3f",
                        i + 1, x_real, y_real, z_real);
            
        }
        // After collecting all points, we proceed with the transformation calculation
        if (!camera_points.empty() && camera_points.size() == real_world_points.size()) {
            // Perform SVD to calculate the transformation matrix
            Eigen::Matrix3d R;  // Rotation matrix
            Eigen::Vector3d t;  // Translation vector

            // Call the helper function to estimate the transformation
            estimateTransform(camera_points, real_world_points, R, t);

            Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");

            std::stringstream ss_r;
            ss_r << R.format(CleanFmt);
            RCLCPP_INFO(this->get_logger(), "Estimated Rotation Matrix:\n%s", ss_r.str().c_str());
            RCLCPP_INFO(this->get_logger(), "Estimated Translation Vector: x: %.3f, y: %.3f, z: %.3f", t.x(), t.y(), t.z());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error: Mismatched number of detected centroids and real-world points.");
        }

        /* // Step 3: Output the centroids
        if (!centroids.empty()) {
            RCLCPP_INFO(this->get_logger(), "Red cylinder centroids detected at:");
            for (const auto& centroid : centroids) {
                RCLCPP_INFO(this->get_logger(), "x: %.3f, y: %.3f, z: %.3f", std::get<0>(centroid), std::get<1>(centroid), std::get<2>(centroid));
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "No clusters found.");
        } */


        // Trigger another capture after processing
        //suggestCaptureSettingsAndCapture();
    }

    void estimateTransform(const std::vector<Eigen::Vector3d>& camera_points, 
                       const std::vector<Eigen::Vector3d>& real_world_points, 
                       Eigen::Matrix3d& R, Eigen::Vector3d& t) 
    {
        // Calculate centroids of both sets of points
        Eigen::Vector3d centroid_camera = Eigen::Vector3d::Zero();
        Eigen::Vector3d centroid_world = Eigen::Vector3d::Zero();

        for (size_t i = 0; i < camera_points.size(); ++i) {
            centroid_camera += camera_points[i];
            centroid_world += real_world_points[i];
        }

        centroid_camera /= camera_points.size();
        centroid_world /= real_world_points.size();

        // Subtract centroids from points
        std::vector<Eigen::Vector3d> camera_centered, world_centered;
        for (size_t i = 0; i < camera_points.size(); ++i) {
            camera_centered.push_back(camera_points[i] - centroid_camera);
            world_centered.push_back(real_world_points[i] - centroid_world);
        }

        // Compute covariance matrix
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < camera_centered.size(); ++i) {
            H += camera_centered[i] * world_centered[i].transpose();
        }

        // Perform Singular Value Decomposition (SVD)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        // Calculate rotation matrix
        R = V * U.transpose();

        // Ensure a right-handed coordinate system
        if (R.determinant() < 0) {
            V.col(2) *= -1;
            R = V * U.transpose();
        }

        // Calculate translation vector
        t = centroid_world - R * centroid_camera;
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
    rclcpp::Client<zivid_interfaces::srv::CaptureAssistantSuggestSettings>::SharedPtr capture_assistant_client_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
