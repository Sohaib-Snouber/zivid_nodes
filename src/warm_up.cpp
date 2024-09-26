#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <thread>
#include <stdexcept>

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

void set_settings(const std::shared_ptr<rclcpp::Node> & node)
{
  RCLCPP_INFO(node->get_logger(), "Setting parameter 'settings_yaml'");

  // YAML settings for the camera
  const std::string settings_yaml =
    R"(
__version__:
  serializer: 1
  data: 22
Settings:
  Acquisitions:
    - Acquisition:
        Aperture: 5.66
        ExposureTime: 8333
  Processing:
    Filters:
      Outlier:
        Removal:
          Enabled: yes
          Threshold: 5
)";

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");
  
  // Wait for the parameter service to become available
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  // Set the settings_yaml parameter
  auto result = param_client->set_parameters({rclcpp::Parameter("settings_yaml", settings_yaml)});
  
  // Wait for the parameter to be set
  if (rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `settings_yaml` parameter");
  }

  RCLCPP_INFO(node->get_logger(), "Successfully set `settings_yaml` parameter");
}

auto create_capture_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("capture");
  
  // Wait for the capture service to become available
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for capture service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "Capture service is available");
  return client;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("zivid_warmup_node");
  RCLCPP_INFO(node->get_logger(), "Zivid warm-up node started");

  // Set camera settings dynamically
  set_settings(node);

  // Create capture client
  auto capture_client = create_capture_client(node);
  
  // Warm-up configuration
  const auto warmup_duration = std::chrono::minutes(10);  // Total warm-up time
  const auto capture_interval = std::chrono::seconds(5);   // Time between captures

  RCLCPP_INFO(node->get_logger(), "Starting warm-up for %ld minutes", warmup_duration.count());

  auto start_time = std::chrono::steady_clock::now();

  // Function to trigger a capture
  auto trigger_capture = [&]() {
    RCLCPP_INFO(node->get_logger(), "Triggering capture");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    capture_client->async_send_request(request);
  };

  // Warm-up loop
  while (std::chrono::steady_clock::now() - start_time < warmup_duration)
  {
    trigger_capture();

    // Sleep for the specified capture interval
    std::this_thread::sleep_for(capture_interval);

    // Log remaining warm-up time
    auto remaining_time = warmup_duration - (std::chrono::steady_clock::now() - start_time);
    RCLCPP_INFO(node->get_logger(), "Remaining warm-up time: %ld seconds", std::chrono::duration_cast<std::chrono::seconds>(remaining_time).count());
  }

  RCLCPP_INFO(node->get_logger(), "Warm-up completed.");

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
