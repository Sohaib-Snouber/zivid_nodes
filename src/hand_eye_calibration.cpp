#include "full_drive/client.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Zivid/Zivid.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Detector.h>
#include <Eigen/Dense>
#include "service_interfaces/srv/gripper_control.hpp"
#include <vector>
#include <sstream>

class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("calibration_node", options), client_actions_(std::make_shared<ClientActions>()) {
        
        // Create a client for the gripper service
        gripper_client_ = this->create_client<service_interfaces::srv::GripperControl>("gripper_control");

        // Wait for the service to be available
        while (!gripper_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for gripper control service to be available...");
        }
        // Initialize Zivid camera
        camera = zivid.connectCamera();

        // Start automatic calibration process
        executeCalibration();
    }

private:
    Zivid::Application zivid;
    Zivid::Camera camera;
    std::vector<Eigen::Matrix4d> calibration_poses_;
    std::shared_ptr<ClientActions> client_actions_;
    rclcpp::Client<service_interfaces::srv::GripperControl>::SharedPtr gripper_client_;

    const int max_attempts = 2;

    // Function to try and repeat actions in case of failure
    bool tryAction(std::function<bool()> action, const std::string& action_name) {
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            if (action()) {
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "Attempt %d of %d for %s failed. Retrying...", attempt + 1, max_attempts, action_name.c_str());
        }
        RCLCPP_ERROR(this->get_logger(), "All %d attempts for %s failed.", max_attempts, action_name.c_str());
        return false;
    }

    geometry_msgs::msg::PoseStamped createRobotCalibrationPose(double x, double y, double z, double qx, double qy, double qz, double qw) {
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.stamp = this->now();
        target_pose.header.frame_id = "world";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        
        target_pose.pose.orientation.w = qw;
        target_pose.pose.orientation.x = qx;
        target_pose.pose.orientation.y = qy;
        target_pose.pose.orientation.z = qz;
        
        return target_pose;
    }

    void convertPosesTo4_4Matrices(double x, double y, double z, double qx, double qy, double qz, double qw) {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = x;
        pose(1, 3) = y;
        pose(2, 3) = z;

        Eigen::Quaterniond quaternion(qx, qy, qz, qw);
        pose.block<3, 3>(0, 0) = quaternion.toRotationMatrix();

        calibration_poses_.push_back(pose);
    }

    bool moveTo(const geometry_msgs::msg::PoseStamped& target_pose, bool constrain, float motion_speed) {
        return client_actions_->moveTo(target_pose, constrain, motion_speed);
    }

    bool moveLinear(const geometry_msgs::msg::PoseStamped& target_pose) {
        return client_actions_->moveLinear(target_pose);
    }

    bool callGripperService(const std::string& command, int position = 0) {
        auto request = std::make_shared<service_interfaces::srv::GripperControl::Request>();
        request->command = command;
        request->position = position;

        auto future = gripper_client_->async_send_request(request, [this, command](rclcpp::Client<service_interfaces::srv::GripperControl>::SharedFuture result) {
            auto response = result.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Gripper command '%s' executed successfully", command.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Gripper command '%s' failed: %s", command.c_str(), response->message.c_str());
            }
        });

        return true;
    }

    void executeCalibration() {
        try {
            std::vector<Zivid::Calibration::HandEyeInput> handEyeInputs;

            std::vector<geometry_msgs::msg::PoseStamped> robot_calibration_poses = {
                createRobotCalibrationPose(0.08544, 0.19088, 0.54515, 0.996671, -0.00059, -0.01546, -0.08002),
                createRobotCalibrationPose(-0.37985, 0.18753, 0.59259, 0.97582, -0.0234, 0.20227, -0.079316),
                createRobotCalibrationPose(-0.320415, 0.489015, 0.51259, 0.97044, 0.02831, 0.1767, 0.161835)
            };

            for (const auto& pose : robot_calibration_poses) {
                convertPosesTo4_4Matrices(
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w
                );
            }

            for (size_t i = 0; i < robot_calibration_poses.size(); ++i) {
                if (!rclcpp::ok()) {
                    RCLCPP_INFO(this->get_logger(), "Shutting down calibration due to SIGINT.");
                    break;
                }

                RCLCPP_INFO(this->get_logger(), "Moving to calibration pose %zu", i + 1);
                if (!tryAction([&]() { return moveTo(robot_calibration_poses[i], false, 0.1); }, "MoveTo")) {
                    RCLCPP_ERROR(this->get_logger(), "MoveTo task failed.");
                    continue;
                }  

                // Capture using Zivid camera
                const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
                    Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
                    Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::seconds{ 10 } }
                };
                try {
                    const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);
                    auto frame = camera.capture(settings);
                    auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);

                    if (detectionResult.valid()) {
                        RCLCPP_INFO(this->get_logger(), "Calibration object detected.");
                        Zivid::Matrix4x4 robotPose = eigenToZividMatrix(calibration_poses_[i]);
                        handEyeInputs.emplace_back(robotPose, detectionResult);  // This should add the valid detection result
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to detect calibration object.");
                    }
                } catch (const Zivid::Exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Zivid SDK error: %s", e.what());
                    return;
                }
            }
            if (handEyeInputs.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No valid hand-eye inputs found for calibration.");
                return;
            }

            if (rclcpp::ok()) {
                auto calibrationResult = Zivid::Calibration::calibrateEyeInHand(handEyeInputs);
                if (calibrationResult.valid()) {
                    RCLCPP_INFO(this->get_logger(), "Hand-eye calibration successful!");
                    RCLCPP_INFO(this->get_logger(), "Transformation Matrix:\n%s", matrixToString(calibrationResult.transform()).c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Hand-eye calibration failed.");
                }
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    Zivid::Matrix4x4 eigenToZividMatrix(const Eigen::Matrix4d &eigen_matrix) {
        Zivid::Matrix4x4 zivid_matrix;
        for (size_t row = 0; row < 4; ++row) {
            for (size_t col = 0; col < 4; ++col) {
                zivid_matrix(row, col) = static_cast<float>(eigen_matrix(row, col));
            }
        }
        return zivid_matrix;
    }

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
