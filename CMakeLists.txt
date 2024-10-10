cmake_minimum_required(VERSION 3.8)
project(zivid_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(zivid_camera REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
# Find Zivid SDK package
find_package(Zivid REQUIRED)
# Client actions dependency (ensure this is linked properly)
find_package(custom_messages REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(full_drive REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(service_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(zivid_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(warm_up src/warm_up.cpp)
ament_target_dependencies(warm_up
 rclcpp 
 std_srvs 
 zivid_camera
)

add_executable(warm_camera src/warm_camera.cpp)
ament_target_dependencies(warm_camera
 rclcpp 
 std_srvs 
 zivid_camera
)

add_executable(robot_pose_node src/robot_pose_node.cpp)
ament_target_dependencies(robot_pose_node
 rclcpp 
 Eigen3
)

add_executable(calibration_node src/calibration_node.cpp)
ament_target_dependencies(calibration_node
 rclcpp 
 sensor_msgs
 Eigen3 
 tf2_geometry_msgs
 tf2_ros
 tf2
 full_drive
 action_interfaces
 service_interfaces
 std_srvs
 cv_bridge
 image_transport
 zivid_interfaces
 pcl_conversions
)

add_executable(hand_eye_calibration src/hand_eye_calibration.cpp)
ament_target_dependencies(hand_eye_calibration
 rclcpp 
 Eigen3 
 Zivid   
 tf2_geometry_msgs
 tf2_ros
 tf2
 full_drive
 action_interfaces
 service_interfaces
)

add_executable(camera_detection src/camera_detection.cpp)
ament_target_dependencies(camera_detection
 rclcpp 
 sensor_msgs
 Eigen3 
 Zivid   
 tf2_geometry_msgs
 tf2_ros
 tf2
 full_drive
 action_interfaces
 service_interfaces
 std_srvs
 cv_bridge
 image_transport
 zivid_interfaces
 pcl_conversions
)
add_executable(testing_node src/testing_node.cpp)
ament_target_dependencies(testing_node
 rclcpp 
 sensor_msgs
 Eigen3 
 Zivid   
 tf2_geometry_msgs
 tf2_ros
 tf2
 full_drive
 cv_bridge
 image_transport
 pcl_conversions
)
add_executable(object_detection src/object_detection.cpp)
ament_target_dependencies(object_detection
 rclcpp 
 sensor_msgs
 Eigen3 
 tf2_geometry_msgs
 tf2_ros
 tf2
 full_drive
 action_interfaces
 service_interfaces
 std_srvs
 cv_bridge
 image_transport
 zivid_interfaces
 pcl_conversions
)
add_executable(own_calibration src/own_calibration.cpp)
ament_target_dependencies(own_calibration
 rclcpp 
 Eigen3 
 tf2_geometry_msgs
)

add_executable(point_cloud_tf src/point_cloud_tf.cpp)
ament_target_dependencies(point_cloud_tf
 rclcpp 
 Eigen3 
 tf2_geometry_msgs
 tf2_ros
 tf2
)

target_link_libraries(calibration_node Zivid::Core)
target_link_libraries(hand_eye_calibration Zivid::Core)

target_link_libraries(camera_detection Zivid::Core)
target_link_libraries(camera_detection ${OpenCV_LIBS})

target_link_libraries(testing_node Zivid::Core)
target_link_libraries(testing_node ${OpenCV_LIBS})

target_link_libraries(object_detection ${OpenCV_LIBS})

target_link_libraries(warm_camera Zivid::Core)

include_directories(${EIGEN3_INCLUDE_DIR})
target_include_directories(calibration_node PUBLIC 
  ${CMAKE_SOURCE_DIR}/../full_drive/include
)
target_include_directories(hand_eye_calibration PUBLIC 
  ${CMAKE_SOURCE_DIR}/../full_drive/include
)
target_include_directories(camera_detection PUBLIC 
  ${CMAKE_SOURCE_DIR}/../full_drive/include
)
target_include_directories(testing_node PUBLIC 
  ${CMAKE_SOURCE_DIR}/../full_drive/include
)
target_include_directories(camera_detection PUBLIC 
  ${CMAKE_SOURCE_DIR}/../full_drive/include
)

install(TARGETS
 warm_up
 warm_camera
 robot_pose_node
 calibration_node
 hand_eye_calibration
 camera_detection
 object_detection
 testing_node
 own_calibration
 point_cloud_tf
 DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

