cmake_minimum_required(VERSION 3.16)
project(drdds)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Keep this list in dependency order for readability; rosidl does not care.
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MetaType.msg"
    # Leg odometry / motion state (input)
    "msg/GaitValue.msg"
    "msg/MotionStateValue.msg"
    "msg/MotionInfoValue.msg"
    "msg/MotionInfo.msg"
    # Localization status (output)
    "msg/ExecStatusValue.msg"
    "msg/LossStatusValue.msg"
    "msg/InputStatusValue.msg"
    "msg/LocationStatusValue.msg"
    "msg/LocationStatus.msg"
    DEPENDENCIES builtin_interfaces
)

ament_package()
