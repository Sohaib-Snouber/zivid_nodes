#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <chrono>

class TransformationPublisher : public rclcpp::Node
{
public:
    TransformationPublisher() : Node("transformation_publisher")
    {
        // Initialize the transformation broadcaster
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Define your rotation matrix and translation vector
        R << 0.997,  0.0627, -0.0412, 
             -0.0613,   0.998,  0.033,
              0.0431, -0.0304,   0.999;
        t << 0.001, -0.022, -0.000;

        // Create a wall timer to publish the transform periodically (every second)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TransformationPublisher::broadcastTransformation, this));
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Class members to store the rotation matrix and translation vector
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    void broadcastTransformation()
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        // Set the parent and child frames
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "optical_center";  // Parent frame
        transformStamped.child_frame_id = "zivid_optical_frame";  // Child frame

        // Translation part
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();

        // Rotation part (convert RPY to quaternion)
        Eigen::Quaterniond q(R);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // Broadcast the transform
        static_broadcaster_->sendTransform(transformStamped);

        RCLCPP_INFO(this->get_logger(), "Transformation broadcasted");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformationPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
