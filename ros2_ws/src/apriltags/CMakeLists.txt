cmake_minimum_required(VERSION 3.8)
project(apriltags)

set(CMAKE_PREFIX_PATH "/usr/local/lib")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic)
  add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(apriltag 3.2 REQUIRED)
# find_package(image_transport REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(apriltag REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(estimate_pose src/estimate_pose.cpp src/test_apriltags.cpp)
target_link_libraries(estimate_pose apriltag::apriltag)
ament_target_dependencies(estimate_pose rclcpp rclcpp_components sensor_msgs tf2_ros image_transport cv_bridge OpenCV)
target_include_directories(estimate_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(estimate_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS estimate_pose
  DESTINATION lib/${PROJECT_NAME})

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
