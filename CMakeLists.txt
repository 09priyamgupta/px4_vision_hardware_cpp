cmake_minimum_required(VERSION 3.8)
project(px4_vision_hardware_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------- Find dependencies ---------------------------------

# CPP dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Python dependencies
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ---------------------------------- Build C++ Executable -----------------------------
# 1. Tell CMake which files make up your executable
add_executable(vision_landing_cpp_exec
  src/main.cpp
  src/mission_executor.cpp
  src/vision_landing_mode.cpp
  src/search_landing_pad_mode.cpp
)

add_executable(landing_director src/landing_director.cpp)

add_executable(apriltag_relative_pose src/apriltag_relative_pose.cpp)

add_executable(landing_data_logger src/landing_data_logger.cpp)

# 2. Tell CMake where to find your .hpp headers
target_include_directories(vision_landing_cpp_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. Link the ROS 2 packages to the executable
ament_target_dependencies(vision_landing_cpp_exec
  rclcpp
  px4_msgs
  std_msgs
  geometry_msgs
  std_srvs
  px4_ros2_cpp
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
)

ament_target_dependencies(landing_director 
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
  std_msgs
  px4_msgs
)

ament_target_dependencies(apriltag_relative_pose 
  rclcpp 
  geometry_msgs 
  std_msgs
  tf2_ros 
  tf2_geometry_msgs
)

ament_target_dependencies(landing_data_logger
  rclcpp
  std_msgs
  geometry_msgs
  px4_msgs
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
)

# 4. Link Eigen3
target_link_libraries(vision_landing_cpp_exec Eigen3::Eigen)

# ---------------------------------- Install ----------------------------------
# Install the C++ executable so 'ros2 launch' can find it
install(TARGETS vision_landing_cpp_exec
  landing_director
  apriltag_relative_pose
  landing_data_logger
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config folders to the share directory
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes so ros2 launch can find them
install(PROGRAMS
  px4_vision_hardware_cpp/gps_rover_enu_publisher.py   
  px4_vision_hardware_cpp/rover_state_ekf_node.py
  px4_vision_hardware_cpp/rover_relative_state_node.py
  # px4_vision_hardware_cpp/landing_director.py
  # px4_vision_hardware_cpp/apriltag_relative_pose.py
  px4_vision_hardware_cpp/mpc_solver_node.py
  px4_vision_hardware_cpp/tag_rviz_markers.py
  tag_detection_analysis/fov_calibration_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------- Build and install ----------------------------------

# Install the Python module (behaviors/utils folders)
# This looks for a folder named 'px4_vision_hardware_cpp' inside your package
ament_python_install_package(${PROJECT_NAME})

# Install your standalone Python nodes (scripts)
# NOTE: You will list files here once you decide which stay in Python
install(PROGRAMS
    # scripts/your_script.py  <-- Add these later
    DESTINATION lib/${PROJECT_NAME}
)

# Handle Testing
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