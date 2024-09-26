#include "full_drive/client.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Zivid/Zivid.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Detector.h>
#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <yaml-cpp/yaml.h>  // For handling YAML settings files
#include "service_interfaces/srv/gripper_control.hpp"

class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("calibration_node", options), client_actions_(std::make_shared<ClientActions>()) {
        
        // Create a service client to control the gripper
        gripper_client_ = this->create_client<service_interfaces::srv::GripperControl>("gripper_control");

        // Wait for the service to be available
        while (!gripper_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for gripper control service to be available...");
        }
        
        // Initialize calibration poses (18 poses in 2 layers: close and far)
        initializeCalibrationPoses();

        // Start automatic calibration process
        executeCalibration();
    }

private:
    std::shared_ptr<ClientActions> client_actions_;
    rclcpp::Client<service_interfaces::srv::GripperControl>::SharedPtr gripper_client_;
    std::vector<Eigen::Matrix4d> calibration_poses_; // Store the transformation matrices
    Zivid::Application zivid;
    Zivid::Camera camera;

    const int max_attempts = 100;

    // Try action with retry mechanism
    bool tryAction(std::function<bool()> action, const std::string &action_name) {
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            if (action()) {
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "Attempt %d of %d for %s failed. Retrying...", attempt + 1, max_attempts, action_name.c_str());
        }
        RCLCPP_ERROR(this->get_logger(), "All %d attempts for %s failed.", max_attempts, action_name.c_str());
        return false;
    }

    // Call gripper service
    bool callGripperService(const std::string &command, int position = 0) {
        auto request = std::make_shared<service_interfaces::srv::GripperControl::Request>();
        request->command = command;
        request->position = position;

        auto future = gripper_client_->async_send_request(request);

        // Wait until the service call is complete
        auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Gripper command '%s' executed successfully", command.c_str());
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Gripper command '%s' failed: %s", command.c_str(), response->message.c_str());
                return false;
            }
        } else if (result == rclcpp::FutureReturnCode::TIMEOUT) {
            RCLCPP_ERROR(this->get_logger(), "Service call to gripper timed out");
            return false;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call to gripper failed");
            return false;
        }
    }

    // Initialize calibration poses
    void initializeCalibrationPoses() {
        // Far Layer (height = 0.7m)
        addCalibrationPose(-0.1853, -0.2072, 0.7464, 0.043, 2.812, -0.192); // Far layer pose 1
        addCalibrationPose(-0.16316, -0.27452, 0.7464, 0.071, 2.99, -0.032); // Far layer pose 2
        addCalibrationPose(-0.16316, -0.13117, 0.7464, 0.041, 2.954, -0.422); // Far layer pose 3
        addCalibrationPose(-0.16316, -0.43368, 0.7464, 0.098, 2.977, 0.319); // Far layer pose 4
        addCalibrationPose(-0.13441, -0.32216, 0.7464, 0, 0, 0); // Far layer pose 5
        
        // Close Layer (height = 0.4m)
        addCalibrationPose(0.458, -0.249, 0.519, 0.021, 3.235, -0.127); // Close layer pose 1
        addCalibrationPose(0.458, -0.508, 0.4775, 0.016, -2.983, -0.612); // Close layer pose 2
        addCalibrationPose(0.1477, -0.2145, 0.4997, 0.010, -2.914, 0.202); // Close layer pose 3
        addCalibrationPose(-0.1853, -0.2072, 0.5215, 0.043, 2.812, -0.192); // Close layer pose 4

        
    }

    // Add calibration pose
    void addCalibrationPose(double x, double y, double z, double rx, double ry, double rz) {
        tf2::Quaternion q;
        q.setRPY(rx, ry, rz);

        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = x;
        pose(1, 3) = y;
        pose(2, 3) = z;

        Eigen::Quaterniond quaternion(q.x(), q.y(), q.z(), q.w());
        pose.block<3, 3>(0, 0) = quaternion.toRotationMatrix();

        calibration_poses_.push_back(pose);
    }

    // Function to execute the automatic calibration
    void executeCalibration() {
        try {
            camera = zivid.connectCamera();
            std::vector<Zivid::Calibration::HandEyeInput> handEyeInputs;

            for (size_t i = 0; i < calibration_poses_.size(); ++i) {
                bool is_close = (calibration_poses_[i](2, 3) < 0.6);  // Determine if the pose is close or far
                RCLCPP_INFO(this->get_logger(), "Moving to pose %zu...", i + 1);

                // Move to the calibration pose
                if (!moveTo(calibration_poses_[i], false, 0.1)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to move to target pose.");
                    return;
                }

                Zivid::Matrix4x4 robotPose = eigenToZividMatrix(calibration_poses_[i]);

                Zivid::Settings settings;
                if (is_close) {
                    RCLCPP_INFO(this->get_logger(), "Using close layer settings");
                    settings = loadSettingsFromYAML("/home/sohaib/zivid_ws/Zivid2_Settings_Zivid_Two_M70_InspectionClose.yml");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Using far layer settings");
                    settings = loadSettingsFromYAML("/home/sohaib/zivid_ws/Zivid2_Settings_Zivid_Two_M70_InspectionFar.yml");
                }

                auto frame = camera.capture(settings);
                auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);

                if (detectionResult.valid()) {
                    RCLCPP_INFO(this->get_logger(), "Calibration object detected.");
                    handEyeInputs.emplace_back(robotPose, detectionResult);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to detect calibration object!");
                    return;
                }
            }

            auto calibrationResult = Zivid::Calibration::calibrateEyeInHand(handEyeInputs);
            if (calibrationResult.valid()) {
                RCLCPP_INFO(this->get_logger(), "Hand-eye calibration successful!");
                RCLCPP_INFO(this->get_logger(), "Transformation Matrix:\n%s", matrixToString(calibrationResult.transform()).c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Hand-eye calibration failed!");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    // Load settings from YAML file
    Zivid::Settings loadSettingsFromYAML(const std::string &yaml_file) {
        try {
            Zivid::Settings settings(yaml_file);  // Load the file by path
            return settings;
        } catch (const std::exception &e) {
            throw std::runtime_error("Failed to load settings from file: " + yaml_file + "\n" + e.what());
        }
    }

    // Move robot to pose using `ClientActions`
    bool moveTo(const Eigen::Matrix4d &target_pose, bool constrain, bool motion_speed) {
        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.stamp = this->now();
        target_pose_msg.header.frame_id = "world";
        target_pose_msg.pose.position.x = target_pose(0, 3);
        target_pose_msg.pose.position.y = target_pose(1, 3);
        target_pose_msg.pose.position.z = target_pose(2, 3);

        // Convert Eigen rotation matrix to tf2::Quaternion
        Eigen::Matrix3d rotation_matrix = target_pose.block<3, 3>(0, 0); // Extract rotation matrix
        tf2::Matrix3x3 tf2_matrix(
            rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
            rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
            rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
        );
        tf2::Quaternion quaternion;
        tf2_matrix.getRotation(quaternion);

        // Set orientation in target_pose_msg
        target_pose_msg.pose.orientation = tf2::toMsg(quaternion);

        return tryAction([&]() { return client_actions_->moveTo(target_pose_msg, constrain, motion_speed); }, "moveTo");
    }

    // Helper function to convert Eigen matrix to Zivid matrix
    Zivid::Matrix4x4 eigenToZividMatrix(const Eigen::Matrix4d &eigen_matrix) {
        Zivid::Matrix4x4 zivid_matrix;
        for (size_t row = 0; row < 4; ++row) {
            for (size_t col = 0; ++col < 4; ++col) {
                zivid_matrix(row, col) = static_cast<float>(eigen_matrix(row, col));
            }
        }
        return zivid_matrix;
    }

    // Helper function to convert Zivid matrix to string for logging
    std::string matrixToString(const Zivid::Matrix4x4 &matrix) {
        std::stringstream ss;
        ss << matrix;
        return ss.str();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}