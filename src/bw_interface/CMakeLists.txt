cmake_minimum_required(VERSION 3.8)
project(bw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 自定义消息/服务所依赖的其他接口包
find_package(builtin_interfaces REQUIRED)

# 列出所有要生成的文件，并通过 DEPENDENCIES 指定它们依赖的其他消息包。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VR.msg"
  "msg/Event.msg"
  "msg/State.msg"
  "msg/TeleopMode.msg"
  "msg/CtrlCmd.msg"
  "msg/ForceFdb.msg"
  
  "srv/StartRecord.srv"
  "srv/PauseRecord.srv"
  "srv/StopRecord.srv"
  "srv/ResumeRecord.srv"
  "srv/GetStatus.srv"
  "srv/SetCtrlSrc.srv"

  "srv/PlaybackLoad.srv"
  "srv/PlaybackSave.srv"
  "srv/PlaybackStart.srv"
  "srv/PlaybackStop.srv"
  "srv/PlaybackRecordFrame.srv"
  "srv/PlaybackClear.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
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

ament_package()
