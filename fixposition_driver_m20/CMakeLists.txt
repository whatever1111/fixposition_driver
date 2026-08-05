# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.16)
project(fixposition_driver_m20)
set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DEPENDENCIES =========================================================================================================
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drdds REQUIRED)

# BUILD ================================================================================================================
add_library(${PROJECT_NAME} SHARED
    src/deskew.cpp
    src/status_monitor.cpp
    src/time_utils.cpp
    src/m20_localization.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Eigen headers are included as SYSTEM so that warnings raised inside them do not
# hit our -Werror. On aarch64 Eigen's NEON path (PacketMath.h) memcpy's its own
# packet wrappers and trips -Wclass-memaccess, which fails the build on the robot
# while x86 never compiles that code at all.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    nav_msgs
    sensor_msgs
    drdds
)

# INSTALL ==============================================================================================================
install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp nav_msgs sensor_msgs drdds Eigen3)

# TESTS ================================================================================================================
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_deskew test/test_deskew.cpp)
    target_link_libraries(test_deskew ${PROJECT_NAME})

    ament_add_gtest(test_status_monitor test/test_status_monitor.cpp)
    target_link_libraries(test_status_monitor ${PROJECT_NAME})

    ament_add_gtest(test_time_utils test/test_time_utils.cpp)
    target_link_libraries(test_time_utils ${PROJECT_NAME})
endif()

ament_package()
