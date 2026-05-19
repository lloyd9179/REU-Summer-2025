cmake_minimum_required(VERSION 3.8)
project(vision-nav-manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(image_pipeline REQUIRED)

# find_package(usb_cam REQUIRED) 
# find_package(usb_cam REQUIRED PATHS /home/cosmos2/tuskegee_robot_ws/src/usb_cam)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

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

install(FILES config/configs.yaml DESTINATION share/${PROJECT_NAME}/config)


# Glob all Python files in the src folder
file(GLOB src_files src/*.py)

# Install all Python files as executables
install(
  PROGRAMS ${src_files}
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
