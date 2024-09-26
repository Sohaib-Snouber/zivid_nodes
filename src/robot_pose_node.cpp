#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

class RobotPoseNode : public rclcpp::Node
{
public:
    RobotPoseNode() : Node("robot_pose_node")
    {
        RCLCPP_INFO(this->get_logger(), "Robot Pose Node started");

        // Prompt the user to input values
        double x = getInputFromUser("Enter X (meters): ");
        double y = getInputFromUser("Enter Y (meters): ");
        double z = getInputFromUser("Enter Z (meters): ");
        double rx = getInputFromUser("Enter Rx (radians): ");
        double ry = getInputFromUser("Enter Ry (radians): ");
        double rz = getInputFromUser("Enter Rz (radians): ");

        RCLCPP_INFO(this->get_logger(), "Received Pose: X=%.5f, Y=%.5f, Z=%.5f, Rx=%.5f, Ry=%.5f, Rz=%.5f",
                    x, y, z, rx, ry, rz);

        // Compute transformation matrix
        Eigen::Matrix4d transformation_matrix = computeTransformationMatrix(x, y, z, rx, ry, rz);
        
        // Display the transformation matrix
        RCLCPP_INFO(this->get_logger(), "Transformation Matrix:\n%s", matrixToString(transformation_matrix).c_str());
    }

private:
    // Convert [X, Y, Z, Rx, Ry, Rz] to a 4x4 transformation matrix
    Eigen::Matrix4d computeTransformationMatrix(double x, double y, double z, double rx, double ry, double rz)
    {
        // Convert from Rx, Ry, Rz to a rotation matrix
        Eigen::AngleAxisd rollAngle(rx, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(ry, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rz, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rotation_matrix = quaternion.matrix();

        // Create a 4x4 transformation matrix
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
        transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
        transformation_matrix(0, 3) = x;
        transformation_matrix(1, 3) = y;
        transformation_matrix(2, 3) = z;

        return transformation_matrix;
    }

    // Helper function to convert Eigen matrix to string for logging
    std::string matrixToString(const Eigen::Matrix4d &matrix)
    {
        std::stringstream ss;
        ss << matrix;
        return ss.str();
    }

    // Function to get input from the user
    double getInputFromUser(const std::string &prompt)
    {
        double value;
        std::cout << prompt;
        std::cin >> value;
        return value;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
