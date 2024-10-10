#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class CheckerboardDetection3D : public rclcpp::Node
{
public:
    CheckerboardDetection3D() : Node("checkerboard_detection_3d")
    {
        // Set up transform listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Set up capture service client
        capture_client_ = this->create_client<std_srvs::srv::Trigger>("capture");
        while (!capture_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for capture service...");
        }
        // Subscribe to the image and point cloud topics
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_color", 10, std::bind(&CheckerboardDetection3D::imageCallback, this, std::placeholders::_1));

        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points/xyzrgba", 10, std::bind(&CheckerboardDetection3D::pointCloudCallback, this, std::placeholders::_1));

        // Subscribe to the camera info topic to get the intrinsic parameters
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/color/camera_info", 10, std::bind(&CheckerboardDetection3D::cameraInfoCallback, this, std::placeholders::_1));

        // Set settings from file and trigger the first capture
        set_settings_from_file();  // Load settings from file and apply to camera
    }

private:
    // Store the latest point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_{new pcl::PointCloud<pcl::PointXYZRGBA>()};
    bool point_cloud_received_ = false;

    // Camera intrinsic parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    void set_settings_from_file()
    {
        // Define the path to the settings file (replace with your path if needed)
        const std::string path_to_settings_yml = "/home/sohaib/Desktop/Zivid2_Settings_Zivid_Two_M70_ParcelsReflective.yml";
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Setting parameter `settings_file_path` to '" << path_to_settings_yml << "'");

        auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "zivid_camera");
        while (!param_client->wait_for_service(std::chrono::seconds(3))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
                throw std::runtime_error("Client interrupted while waiting for service to appear.");
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the parameters client to appear...");
        }

        auto result =
            param_client->set_parameters({rclcpp::Parameter("settings_file_path", path_to_settings_yml)});
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::seconds(30)) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set `settings_file_path` parameter");
            throw std::runtime_error("Failed to set `settings_file_path` parameter");
        }

        RCLCPP_INFO(this->get_logger(), "Settings file applied successfully.");

        triggerCapture();
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
            RCLCPP_INFO(this->get_logger(), "Capture service called successfully.");
        }
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Store the intrinsic parameters from the camera info message
        camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data());
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, (void*)msg->d.data());

        // Notify when camera info is received
        RCLCPP_INFO(this->get_logger(), "Camera info received. Intrinsic parameters loaded.");
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 message to PCL point cloud
        pcl::fromROSMsg(*msg, *point_cloud_);
        point_cloud_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Point cloud received.");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (!point_cloud_received_) {
            RCLCPP_WARN(this->get_logger(), "No point cloud received yet. Skipping frame.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Image received");

        // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Undistort the image using the intrinsic parameters
        cv::Mat undistorted_image;
        if (!camera_matrix_.empty() && !dist_coeffs_.empty()) {
            cv::undistort(cv_ptr->image, undistorted_image, camera_matrix_, dist_coeffs_);
            RCLCPP_INFO(this->get_logger(), "Image undistorted.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Camera info not yet received. Using raw image.");
            undistorted_image = cv_ptr->image;
        }

        // Invert the image colors (make black white and white black)
        cv::Mat inverted_image;
        cv::bitwise_not(undistorted_image, inverted_image);

        // Detect the checkerboard corners in 2D
        cv::Size pattern_size(7, 6);  // Size of checkerboard (number of internal corners)
        std::vector<cv::Point2f> corners_2d;  // Store the detected 2D corners

        bool pattern_found = cv::findChessboardCorners(inverted_image, pattern_size, corners_2d,
                                                       cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (pattern_found) {
            RCLCPP_INFO(this->get_logger(), "Checkerboard pattern found!");

            // Draw the detected corners on the image
            cv::drawChessboardCorners(undistorted_image, pattern_size, cv::Mat(corners_2d), pattern_found);

            // Save the image to a file (e.g., your desktop)
            std::string output_file = "/home/sohaib/Desktop/second_layer_detected_corners_inverted.png";
            if (cv::imwrite(output_file, undistorted_image)) {
                RCLCPP_INFO(this->get_logger(), "Image saved to: %s", output_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image.");
            }
            // Find the corresponding 3D points for each detected 2D corner
            std::vector<cv::Point3f> corners_3d;
            for (const auto& corner_2d : corners_2d) {
                cv::Point3f corner_3d = get3DPoint(corner_2d.x, corner_2d.y);
                corners_3d.push_back(corner_3d);
                RCLCPP_INFO(this->get_logger(), "3D Corner: x=%.3f, y=%.3f, z=%.3f", corner_3d.x, corner_3d.y, corner_3d.z);
                // Step 3: Transform centroids from camera frame to world frame
                std::string target_frame = "world";  // Replace this with your world frame
                std::string source_frame = "zivid_optical_frame";  // Frame of the Zivid camera

                geometry_msgs::msg::PointStamped point_camera_frame;
                point_camera_frame.header.frame_id = source_frame;
                point_camera_frame.point.x = corner_3d.x;
                point_camera_frame.point.y = corner_3d.y;
                point_camera_frame.point.z = corner_3d.z;

                try {
                    auto transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                    geometry_msgs::msg::PointStamped point_world_frame;
                    tf2::doTransform(point_camera_frame, point_world_frame, transform);
                    RCLCPP_INFO(this->get_logger(), "Red cylinder centroid in world frame: x: %.3f, y: %.3f, z: %.3f",
                                point_world_frame.point.x, point_world_frame.point.y, point_world_frame.point.z);
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Checkerboard pattern not found.");
        }
    }

    // Function to get the 3D point corresponding to a 2D point from the point cloud
    cv::Point3f get3DPoint(float u, float v)
    {
        // Ensure the point cloud is available and valid
        if (!point_cloud_ || point_cloud_->empty()) {
            return cv::Point3f(0, 0, 0);  // Return a dummy point if no point cloud
        }

        // Find the corresponding 3D point in the point cloud (nearest neighbor interpolation)
        int idx = static_cast<int>(v) * point_cloud_->width + static_cast<int>(u);

        if (idx < 0 || idx >= point_cloud_->points.size()) {
            RCLCPP_WARN(this->get_logger(), "Point outside point cloud bounds");
            return cv::Point3f(0, 0, 0);  // Invalid point
        }

        pcl::PointXYZRGBA pcl_point = point_cloud_->points[idx];
        return cv::Point3f(pcl_point.x, pcl_point.y, pcl_point.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CheckerboardDetection3D>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
