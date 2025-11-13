cmake_minimum_required(VERSION 3.10)
project(nero_drone)

# ---------------------------------------------------------
# Dependencies/Packages
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ---------------------------------------------------------
# Executables
# ---------------------------------------------------------

# ---------------- C++ Nodes ----------------
# Joy2Cmd
add_executable(Joy2Cmd src/Joy2Cmd.cpp)
# ament_target_dependencies(Joy2Cmd rclcpp sensor_msgs geometry_msgs std_msgs)
# target_link_libraries(Joy2Cmd ${OpenCV_LIBS})
target_link_libraries(Joy2Cmd PUBLIC
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

# apriltag_node
add_executable(apriltag_node src/apriltag_node.cpp)
# ament_target_dependencies(apriltag_node rclcpp)
# target_link_libraries(apriltag_node ${OpenCV_LIBS} apriltag)
target_link_libraries(apriltag_node PUBLIC
  ${OpenCV_LIBS}
  rclcpp::rclcpp
  apriltag
)

# beboptag
add_executable(beboptag src/beboptag.cpp)
# ament_target_dependencies(beboptag rclcpp sensor_msgs cv_bridge)
# target_link_libraries(beboptag ${OpenCV_LIBS} apriltag)
target_link_libraries(beboptag PUBLIC
  ${sensor_msgs_TARGETS}
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  apriltag
)

# tf_cam
add_executable(tf_cam src/tf_cam.cpp)
# ament_target_dependencies(tf_cam rclcpp tf2 tf2_ros geometry_msgs nav_msgs std_msgs tf2_geometry_msgs)
target_link_libraries(tf_cam PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

# tf_tag_bebop
add_executable(tf_tag_bebop src/tf_tag_bebop.cpp)
# ament_target_dependencies(tf_tag_bebop
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   tf2_ros
#   geometry_msgs
#   visualization_msgs
#   tf2
#   tf2_geometry_msgs
# )
target_link_libraries(tf_tag_bebop PUBLIC
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  apriltag
)

# RefPublisher
add_executable(RefPublisher src/RefPublisher.cpp)
# ament_target_dependencies(RefPublisher rclcpp geometry_msgs)
target_link_libraries(RefPublisher PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

# velocity_logger
add_executable(velocity_logger src/velocity_logger.cpp)
# ament_target_dependencies(velocity_logger
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   std_msgs
#   tf2
#   tf2_geometry_msgs
# )
target_link_libraries(velocity_logger PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# ref_vec_filter
add_executable(ref_vec_filter src/filtred.cpp)
# ament_target_dependencies(ref_vec_filter
#   rclcpp
#   geometry_msgs
#   std_msgs
#   visualization_msgs
#   tf2_geometry_msgs
# )
target_link_libraries(ref_vec_filter PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
)

# tagfollow
add_executable(tagfollow src/tagfollow.cpp)
# ament_target_dependencies(tagfollow
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   nav_msgs
#   std_msgs
#   tf2
#   tf2_ros
#   visualization_msgs
# )
target_link_libraries(tagfollow PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

# visual
add_executable(visual src/visual.cpp)
# ament_target_dependencies(visual
#   rclcpp
#   geometry_msgs
#   visualization_msgs
#   tf2_ros
# )
target_link_libraries(visual PUBLIC
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

# land
add_executable(land src/land.cpp)
# ament_target_dependencies(land rclcpp std_msgs)
target_link_libraries(land PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# safety_watchdog
add_executable(safety_watchdog src/safe_watchdog.cpp)
# ament_target_dependencies(safety_watchdog rclcpp geometry_msgs std_msgs)
target_link_libraries(safety_watchdog PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# safe_bebop_republisher
add_executable(safe_bebop_republisher src/bridge_cmd.cpp)
# ament_target_dependencies(safe_bebop_republisher rclcpp geometry_msgs)
target_link_libraries(safe_bebop_republisher PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

# isfly
add_executable(isfly src/isfly.cpp)
# ament_target_dependencies(isfly rclcpp std_msgs)
target_link_libraries(isfly PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)


# ---------------------------------------------------------
# Python nodes
# ---------------------------------------------------------
# ---------------- Instalação ----------------
install(TARGETS
  Joy2Cmd
  apriltag_node
  beboptag
  tf_cam
  tf_tag_bebop
  velocity_logger
  tagfollow
  visual
  safety_watchdog
  safe_bebop_republisher
  land
  ref_vec_filter
  RefPublisher
  isfly
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Python nodes ----------------
install(PROGRAMS
  nero_drone/graphical.py
  nero_drone/apriltag.py
  nero_drone/takeoff.py
  nero_drone/land.py
  nero_drone/calibration_publish.py
  nero_drone/bebop_control_gui.py
  nero_drone/ref_pos.py
  nero_drone/ref_trayectory.py
  nero_drone/controlador.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Launch, Config, Assets ----------------
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf meshes others DESTINATION share/${PROJECT_NAME})

ament_package()

