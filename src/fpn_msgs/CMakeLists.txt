cmake_minimum_required(VERSION 3.8)
project(fpn_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages and services          #
################################################

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FourWD.msg"
  "msg/FullGPS.msg"
  "msg/Goal.msg"
  "msg/GPSPosition.msg"
  "msg/GoalMultiArray.msg"
  "msg/HybridPerception.msg"
  "msg/LidarAnalysis.msg"
  "msg/LidarGapData.msg"
  "msg/Motor.msg"
  "msg/MPCInput.msg"
  "msg/MPCOutput.msg"
  "msg/MHEOutput.msg"
  "msg/PerceptionLidarController.msg"
  "msg/PerceptionLidarControllerDebug.msg"
  "msg/PerceptionLidarLine.msg"
  "msg/PerceptionLidarRow.msg"
  "msg/PerceptionLidar.msg"
  "msg/PID.msg"
  "msg/Terrabot.msg"
  "msg/TSPath.msg"
  "msg/TSWaypoint.msg"
  "srv/BoolSrv.srv"
  "srv/GoStraight.srv"
  "srv/Maneuver.srv"
  "srv/Report.srv"
  "srv/RosbagDetails.srv"
  "srv/TurnDegrees.srv"
  "srv/ZZsetString.srv"
  DEPENDENCIES geometry_msgs
  DEPENDENCIES std_msgs
  DEPENDENCIES nav_msgs
  DEPENDENCIES sensor_msgs
  )

# # Generate services in the 'srv' folder
# rosidl_generate_interfaces(${PROJECT_NAME}

# )

###################################
## Export dependencies and target #
###################################

ament_export_dependencies(rosidl_default_runtime)

###################################
## Install target files #
###################################

# Install the msg and srv files
install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark the package for ament
ament_package()
