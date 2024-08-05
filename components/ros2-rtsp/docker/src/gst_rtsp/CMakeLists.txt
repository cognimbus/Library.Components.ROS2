cmake_minimum_required(VERSION 3.8)
project(gst_rtsp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(gst_rtsp_node src/gst_rtsp_node.cpp)
target_include_directories(gst_rtsp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GSTREAMER_INCLUDE_DIRS}
  $<INSTALL_INTERFACE:include>)
  

target_link_libraries(gst_rtsp_node
  ${GSTREAMER_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(gst_rtsp_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_compile_features(gst_rtsp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS gst_rtsp_node
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
