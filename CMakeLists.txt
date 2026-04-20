cmake_minimum_required(VERSION 3.8)
project(dexi_color_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dexi_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# ── C++ color detection node ──────────────────────────────────────
add_executable(color_detection_node src/color_detection_node.cpp)
target_include_directories(color_detection_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(color_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  dexi_interfaces
)
target_link_libraries(color_detection_node ${OpenCV_LIBS})

install(TARGETS color_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python simulator (kept for desktop testing without a camera) ──
install(PROGRAMS
  src/color_detection_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch + config files ─────────────────────────────────────────
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
