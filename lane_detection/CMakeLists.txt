cmake_minimum_required(VERSION 3.8)
project(lane_detection)

# GCC veya Clang derleyicisi kullanılıyorsa, derleme uyarılarını etkinleştir
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake'yi buluyoruz, bu ROS 2'yi derlemek için gereklidir
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# OpenCV başlık dosyalarını dahil et
include_directories(${OpenCV_INCLUDE_DIRS})

# lane_detection_node adlı çalıştırılabilir dosyayı oluşturuyoruz
add_executable(lane_detection_node src/lane_detection.cpp)


ament_target_dependencies(lane_detection_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  carla_msgs
  std_msgs
)

add_executable(pure_pursuit_controller
  src/pure_pursuit_controller.cpp
)
ament_target_dependencies(pure_pursuit_controller
  rclcpp
  std_msgs
  sensor_msgs 
  nav_msgs
  carla_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
 lane_detection_node 
 pure_pursuit_controller
DESTINATION lib/${PROJECT_NAME})

# Test yapıyorsanız lint bağımlılıklarını kontrol et
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament paketini oluşturuyoruz
ament_package()
