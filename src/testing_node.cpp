#include <rclcpp/rclcpp.hpp>
#include <Zivid/Zivid.h>
#include <Zivid/Calibration/DetectionResult.h>
#include <Zivid/Calibration/Detector.h>

class CalibrationNode : public rclcpp::Node
{
public:
    CalibrationNode(Zivid::Camera &camera) : Node("calibration_node")
    {
        try
        {
            camera_ = camera;

            const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
                Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
                Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 10000 } }
            };

            auto settings = Zivid::CaptureAssistant::suggestSettings(camera_, suggestSettingsParameters);

            // Capture calibration board using Zivid camera
            auto frame = Zivid::Calibration::captureCalibrationBoard(camera_);

            // Detect calibration board feature points in the captured frame
            auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);

            // Check if detection was successful
            if (detectionResult)
            {
                RCLCPP_INFO(this->get_logger(), "Feature points detected successfully.");

                // Print the centroid of detected feature points
                auto centroid = detectionResult.centroid();
                RCLCPP_INFO(this->get_logger(), "Detected centroid: [x: %.3f, y: %.3f, z: %.3f]",
                            centroid.x, centroid.y, centroid.z);

                // Get the transformation matrix (4x4) of the detected pose
                auto poseMatrix = detectionResult.pose().toMatrix();

                // Extract the translation components (last column of the matrix)
                double tx = poseMatrix(0, 3);
                double ty = poseMatrix(1, 3);
                double tz = poseMatrix(2, 3);

                // Log the translation part of the pose
                RCLCPP_INFO(this->get_logger(), "Pose translation: [x: %.3f, y: %.3f, z: %.3f]", tx, ty, tz);

                // You can also log the full pose matrix if needed
                RCLCPP_INFO(this->get_logger(), "Pose matrix:\n[%.3f, %.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f, %.3f]",
                            poseMatrix(0, 0), poseMatrix(0, 1), poseMatrix(0, 2), poseMatrix(0, 3),
                            poseMatrix(1, 0), poseMatrix(1, 1), poseMatrix(1, 2), poseMatrix(1, 3),
                            poseMatrix(2, 0), poseMatrix(2, 1), poseMatrix(2, 2), poseMatrix(2, 3),
                            poseMatrix(3, 0), poseMatrix(3, 1), poseMatrix(3, 2), poseMatrix(3, 3));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Feature point detection failed.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

private:
    Zivid::Camera camera_;


};

int main(int argc, char *argv[])
{
    Zivid::Application zivid;
    auto camera = zivid.connectCamera();

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CalibrationNode>(camera);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
