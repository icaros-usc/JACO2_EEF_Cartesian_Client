cmake_minimum_required(VERSION 3.8)
project(ada_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ada_hardware SHARED
  src/jaco2.cpp
  src/isaacsim.cpp)

set(KINOVA_SDK_ROOT "" CACHE PATH "Kinova Gen2 SDK installation prefix")
find_path(KINOVA_API_INCLUDE_DIR Kinova.API.UsbCommandLayerUbuntu.h
  HINTS "${KINOVA_SDK_ROOT}" ENV KINOVA_SDK_ROOT PATH_SUFFIXES include)
find_library(KINOVA_COMM_LIBRARY NAMES Kinova.API.CommLayerUbuntu.so
  HINTS "${KINOVA_SDK_ROOT}" ENV KINOVA_SDK_ROOT PATH_SUFFIXES lib lib64)
find_library(KINOVA_USB_LIBRARY NAMES Kinova.API.USBCommandLayerUbuntu.so
  HINTS "${KINOVA_SDK_ROOT}" ENV KINOVA_SDK_ROOT PATH_SUFFIXES lib lib64)
if(NOT KINOVA_API_INCLUDE_DIR OR NOT EXISTS "${KINOVA_API_INCLUDE_DIR}/KinovaTypes.h"
    OR NOT KINOVA_COMM_LIBRARY OR NOT KINOVA_USB_LIBRARY)
  message(FATAL_ERROR "Kinova Gen2 SDK missing. Install its headers/libraries first; see DEPLOYMENT.md. For a nonstandard install set KINOVA_SDK_ROOT to its prefix.")
endif()
target_include_directories(ada_hardware PUBLIC include PRIVATE "${KINOVA_API_INCLUDE_DIR}")
set_target_properties(ada_hardware PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

target_link_libraries(ada_hardware "${KINOVA_COMM_LIBRARY}" "${KINOVA_USB_LIBRARY}")

ament_target_dependencies(ada_hardware rclcpp rclcpp_lifecycle pluginlib hardware_interface sensor_msgs)

pluginlib_export_plugin_description_file(hardware_interface ada_hardware.xml)

# Install (required for pluginlib)
install(
  DIRECTORY include/
  DESTINATION include/ada_hardware
)
install(TARGETS ada_hardware
  EXPORT export_ada_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ### Linting
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ### GMock
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_jaco2 test/test_jaco2.cpp)
  ament_target_dependencies(test_jaco2
    pluginlib
    ros2_control_test_assets
    rclcpp
    rclcpp_lifecycle
    hardware_interface
  )

  ament_add_gmock(test_isaacsim test/test_isaacsim.cpp)
  ament_target_dependencies(test_isaacsim
    pluginlib
    ros2_control_test_assets
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    sensor_msgs
  )
endif()

ament_export_dependencies(rclcpp rclcpp_lifecycle pluginlib hardware_interface)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
