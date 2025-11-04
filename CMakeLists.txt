cmake_minimum_required(VERSION 3.5)
project(nero_drone)

# ---------------- Dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# ---------------- Includes ----------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ---------------- Executables C++ ----------------

add_executable(hello_cpp src/hello_cpp.cpp)
ament_target_dependencies(hello_cpp rclcpp)

add_executable(Joy2Cmd src/Joy2Cmd.cpp)
ament_target_dependencies(Joy2Cmd rclcpp sensor_msgs geometry_msgs std_msgs)
target_link_libraries(Joy2Cmd ${OpenCV_LIBS})

add_executable(apriltag_node src/apriltag_node.cpp)
ament_target_dependencies(apriltag_node rclcpp OpenCV)
target_link_libraries(apriltag_node apriltag ${OpenCV_LIBS})

add_executable(KalmanNode src/KalmanNode.cpp)
ament_target_dependencies(KalmanNode rclcpp tf2_ros geometry_msgs OpenCV)
target_link_libraries(KalmanNode ${OpenCV_LIBS})

add_executable(beboptag src/beboptag.cpp)
ament_target_dependencies(beboptag rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(beboptag apriltag ${OpenCV_LIBS})

add_executable(tf_cam src/tf_cam.cpp)
ament_target_dependencies(tf_cam rclcpp tf2 tf2_ros geometry_msgs nav_msgs std_msgs)

add_executable(tf_tag_bebop src/tf_tag_bebop.cpp)
ament_target_dependencies(tf_tag_bebop rclcpp sensor_msgs cv_bridge OpenCV tf2_ros geometry_msgs visualization_msgs tf2 tf2_geometry_msgs)
target_link_libraries(tf_tag_bebop apriltag ${OpenCV_LIBS})

add_executable(VelToControlInput src/VelToControlInput.cpp)
ament_target_dependencies(VelToControlInput rclcpp geometry_msgs)

add_executable(VelocityControllerBody src/VelocityControllerBody.cpp)
ament_target_dependencies(VelocityControllerBody rclcpp geometry_msgs nav_msgs tf2_ros)
target_link_libraries(VelocityControllerBody Eigen3::Eigen)

add_executable(RefPublisher src/RefPublisher.cpp)
ament_target_dependencies(RefPublisher rclcpp geometry_msgs)

# --- 💾 Nuevo ejecutable para logger ---
add_executable(velocity_logger src/velocity_logger.cpp)
ament_target_dependencies(velocity_logger
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(tagfollow src/tagfollow.cpp)
ament_target_dependencies(tagfollow
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
add_executable(visual src/visual.cpp)
ament_target_dependencies(visual
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
)
add_executable(land src/land.cpp)
ament_target_dependencies(land rclcpp std_msgs)

add_executable(inverse_dynamic_controller src/InverseDynamicController.cpp)
ament_target_dependencies(inverse_dynamic_controller
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)


add_executable(poseref src/poseref.cpp)
ament_target_dependencies(poseref
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(safety_watchdog
  src/safe_watchdog.cpp
)
ament_target_dependencies(safety_watchdog
  rclcpp
  geometry_msgs
  std_msgs
)
add_executable(position_control src/position_control.cpp)

# Dependencias del ejecutable
ament_target_dependencies(position_control
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
add_executable(safe_bebop_republisher src/bridge_cmd.cpp)
ament_target_dependencies(safe_bebop_republisher rclcpp geometry_msgs)

# ---------------- Install ----------------
install(TARGETS
  hello_cpp
  Joy2Cmd
  apriltag_node
  beboptag
  tf_cam
  tf_tag_bebop
  VelToControlInput
  VelocityControllerBody
  RefPublisher
  KalmanNode
  velocity_logger
  tagfollow
  visual
  inverse_dynamic_controller
  poseref
  safety_watchdog
  position_control
  safe_bebop_republisher
  land
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Python nodes ----------------
install(PROGRAMS
  nero_drone/hello_py.py
  nero_drone/apriltag.py
  nero_drone/takeoff.py
  nero_drone/land.py
  nero_drone/demo_takeoff_land.py
  nero_drone/calibration_publish.py
  nero_drone/bebop_control_gui.py
  nero_drone/pid_tunning.py
  nero_drone/vrx.py
  nero_drone/inv_vrx.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Launch files ----------------
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ---------------- Config files ----------------
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ---------------- URDF + meshes + others ----------------
install(DIRECTORY urdf meshes others DESTINATION share/${PROJECT_NAME})

ament_package()
