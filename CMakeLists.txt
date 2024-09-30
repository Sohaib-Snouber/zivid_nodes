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

add_executable(warm_up src/warm_up.cpp)
ament_target_dependencies(warm_up rclcpp std_srvs zivid_camera)

add_executable(warm_camera src/warm_camera.cpp)
ament_target_dependencies(warm_camera rclcpp std_srvs zivid_camera)

add_executable(robot_pose_node src/robot_pose_node.cpp)
ament_target_dependencies(robot_pose_node rclcpp Eigen3)

add_executable(calibration_node src/calibration_node.cpp)
ament_target_dependencies(calibration_node
 rclcpp 
 Eigen3 
 Zivid   
 tf2_geometry_msgs
 tf2_ros
 tf2
 custom_messages
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

target_link_libraries(calibration_node Zivid::Core)
target_link_libraries(hand_eye_calibration Zivid::Core)
target_link_libraries(warm_camera Zivid::Core)

include_directories(${EIGEN3_INCLUDE_DIR})
target_include_directories(calibration_node PUBLIC 
  ${CMAKE_SOURCE_DIR}/../full_drive/include
)
target_include_directories(hand_eye_calibration PUBLIC 
  ${CMAKE_SOURCE_DIR}/../full_drive/include
)

install(TARGETS
 warm_up
 warm_camera
 robot_pose_node
 calibration_node
 hand_eye_calibration
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

