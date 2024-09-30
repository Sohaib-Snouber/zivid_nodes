#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Zivid/Zivid.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Detector.h>
#include <Eigen/Dense>
#include "custom_messages/msg/motion_request.hpp"
#include <vector>

class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("calibration_node", options) {
        
        // Create a publisher to send motion requests to the MotionNode
        motion_request_publisher_ = this->create_publisher<custom_messages::msg::MotionRequest>("motion_request", 10);

        // Initialize camera
        camera = zivid.connectCamera();

        // Start automatic calibration process
        executeCalibration();
    }

private:
    rclcpp::Publisher<custom_messages::msg::MotionRequest>::SharedPtr motion_request_publisher_;
    Zivid::Application zivid;
    Zivid::Camera camera;
    std::vector<Eigen::Matrix4d> calibration_poses_;

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

    void sendMotionRequest(const std::string &action_type, const geometry_msgs::msg::PoseStamped &target_pose, bool constrain = false, float motion_speed = 0.1) {
        custom_messages::msg::MotionRequest request_msg;
        request_msg.action_type = action_type;
        request_msg.target_pose = target_pose;
        request_msg.constrain = constrain;
        request_msg.motion_speed = motion_speed;

        try {
            motion_request_publisher_->publish(request_msg);
            RCLCPP_INFO(this->get_logger(), "Sent %s request to MotionNode", action_type.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send motion request: %s", e.what());
        }
    }

    // Function to execute the automatic calibration
    void executeCalibration() {
        try {
            std::vector<Zivid::Calibration::HandEyeInput> handEyeInputs;

            std::vector<geometry_msgs::msg::PoseStamped> robot_calibration_poses;

            /* // Define calibration poses for the robot
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.1853, -0.2072, 0.7464, 0.043, 2.812, -0.192)); // Pose 1
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.16316, -0.27452, 0.7464, 0.071, 2.99, -0.032)); // Pose 2
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.16316, -0.13117, 0.7464, 0.041, 2.954, -0.422)); // Pose 3
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.16316, -0.43368, 0.7464, 0.098, 2.977, 0.319)); // Pose 4
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.13441, -0.32216, 0.7464, 0, 0, 0)); // Pose 5
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.458, -0.249, 0.519, 0.021, 3.235, -0.127)); // Pose 6
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.458, -0.508, 0.4775, 0.016, -2.983, -0.612)); // Pose 7
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.1477, -0.2145, 0.4997, 0.010, -2.914, 0.202)); // Pose 8
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.1853, -0.2072, 0.5215, 0.043, 2.812, -0.192)); // Pose 9 */
            
            // Define calibration poses for the robot
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.08544, 0.19088, 0.54515, 0.996671, -0.00059, -0.01546, -0.08002)); // Pose 1
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.08544, 0.19088, 0.55515, 0.996671, -0.00059, -0.01546, -0.08002)); // Pose 1
            /* robot_calibration_poses.push_back(createRobotCalibrationPose(0.31207, 0.19959, 0.48735, 0.97663, 0.01425, -0.199948, -0.07862)); // Pose 2
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.328, 0.20638, 0.47468, 0.96711, 0.02897, 0.2443, -0.0645)); // Pose 3
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.37985, 0.18753, 0.59259, 0.97582, -0.0234, 0.20227, -0.079316)); // Pose 4
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.320415, 0.489015, 0.51259, 0.97044, 0.02831, 0.1767, 0.161835)); // Pose 5
            robot_calibration_poses.push_back(createRobotCalibrationPose(-0.05647, 0.4982, 0.5411, 0.984833, 0.00819, 0.05582, 0.16407)); // Pose 6
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.15184, 0.49928, 0.54578, 0.973477, -0.02756, -0.15922, 0.16196)); // Pose 7
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.16935, 0.47358, 0.679226, 0.993687, -0.01902, -0.07153, 0.08428)); // Pose 8
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.15322, 0.38491, 0.694855, 0.99622, -0.01372, -0.0088, 0.0853)); // Pose 9
            robot_calibration_poses.push_back(createRobotCalibrationPose(0.15297, 0.34161, 0.7619, 0.999812, -0.0145, -0.00746, -0.01049)); // Pose 10 */

            // Convert these poses to 4x4 matrices for Zivid calibration
            for (const auto& pose : robot_calibration_poses) {
                // Extract position and orientation from the pose
                double x = pose.pose.position.x;
                double y = pose.pose.position.y;
                double z = pose.pose.position.z;

                double qx = pose.pose.orientation.x;
                double qy = pose.pose.orientation.y;
                double qz = pose.pose.orientation.z;
                double qw = pose.pose.orientation.w;


                // Convert to 4x4 matrix with proper rotation and translation
                convertPosesTo4_4Matrices(x, y, z, qx, qy, qz, qw);
            }

            for (size_t i = 0; i < robot_calibration_poses.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Moving to calibration pose %zu", i + 1);

                // Send moveTo request to MotionNode
                sendMotionRequest("MoveTo", robot_calibration_poses[i], false, 0.1);

                // Add delay to allow MotionNode to process the moveTo request
                rclcpp::sleep_for(std::chrono::seconds(5));

                // Do calibration tasks here, e.g., Zivid camera capture and board detection
                bool is_close = calibration_poses_[i](2, 3) < 0.6;
                Zivid::Settings settings;
                if (is_close) {
                    RCLCPP_INFO(this->get_logger(), "Using close layer settings");
                    settings = loadSettingsFromYAML("/home/sohaib/zivid_ws/Zivid2_Settings_Zivid_Two_M70_InspectionClose.yml");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Using far layer settings");
                    settings = loadSettingsFromYAML("/home/sohaib/zivid_ws/Zivid2_Settings_Zivid_Two_M70_InspectionFar.yml");
                }

                auto frame = camera.capture(settings);
                // Add delay to allow MotionNode to process the moveTo request
                rclcpp::sleep_for(std::chrono::seconds(2));
                auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);
                if (detectionResult.valid()) {
                    RCLCPP_INFO(this->get_logger(), "Calibration object detected.");
                    Zivid::Matrix4x4 robotPose = eigenToZividMatrix(calibration_poses_[i]);
                    handEyeInputs.emplace_back(robotPose, detectionResult);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to detect calibration object.");
                }
                // Add delay to allow MotionNode to process the moveTo request
                rclcpp::sleep_for(std::chrono::seconds(1));
            }

            auto calibrationResult = Zivid::Calibration::calibrateEyeInHand(handEyeInputs);
            if (calibrationResult.valid()) {
                RCLCPP_INFO(this->get_logger(), "Hand-eye calibration successful!");
                RCLCPP_INFO(this->get_logger(), "Transformation Matrix:\n%s", matrixToString(calibrationResult.transform()).c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Hand-eye calibration failed.");
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    Zivid::Settings loadSettingsFromYAML(const std::string &yaml_file) {
        try {
            Zivid::Settings settings(yaml_file);  // Load the file by path
            return settings;
        } catch (const std::exception &e) {
            throw std::runtime_error("Failed to load settings from file: " + yaml_file + "\n" + e.what());
        }
    }

    // Helper function to convert Eigen matrix to Zivid matrix
    Zivid::Matrix4x4 eigenToZividMatrix(const Eigen::Matrix4d &eigen_matrix) {
        Zivid::Matrix4x4 zivid_matrix;
        for (size_t row = 0; row < 4; ++row) {
            for (size_t col = 0; col < 4; ++col) {
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