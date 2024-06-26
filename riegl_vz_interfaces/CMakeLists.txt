cmake_minimum_required(VERSION 3.8)
project(riegl_vz_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScanPose.msg"
  "msg/TiePoint.msg"
  "msg/Voxels.msg"
  "srv/GetPointCloud.srv"
  "srv/GetVoxels.srv"
  "srv/GetScanPoses.srv"
  "srv/GetTpl.srv"
  "srv/GetPose.srv"
  "srv/SetPosition.srv"
  "srv/SetPose.srv"
  "srv/GetList.srv"
  "srv/TransformCoord.srv"
  "srv/Command.srv"
  "srv/MoveFile.srv"
  DEPENDENCIES sensor_msgs geometry_msgs
 )

ament_package()
