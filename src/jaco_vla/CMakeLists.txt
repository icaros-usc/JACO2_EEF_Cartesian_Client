cmake_minimum_required(VERSION 3.8)
project(jaco_vla)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jaco_vla_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

set(KINOVA_SDK_ROOT "" CACHE PATH "Kinova Gen2 SDK installation prefix")
find_path(KINOVA_API_INCLUDE_DIR Kinova.API.UsbCommandLayerUbuntu.h
  HINTS "${KINOVA_SDK_ROOT}" ENV KINOVA_SDK_ROOT
  PATH_SUFFIXES include)
find_library(KINOVA_COMM_LIBRARY NAMES Kinova.API.CommLayerUbuntu.so
  HINTS "${KINOVA_SDK_ROOT}" ENV KINOVA_SDK_ROOT
  PATH_SUFFIXES lib lib64)
find_library(KINOVA_USB_LIBRARY NAMES Kinova.API.USBCommandLayerUbuntu.so
  HINTS "${KINOVA_SDK_ROOT}" ENV KINOVA_SDK_ROOT
  PATH_SUFFIXES lib lib64)
if(NOT KINOVA_API_INCLUDE_DIR OR NOT EXISTS "${KINOVA_API_INCLUDE_DIR}/KinovaTypes.h"
    OR NOT KINOVA_COMM_LIBRARY OR NOT KINOVA_USB_LIBRARY)
  message(FATAL_ERROR
    "Kinova Gen2 SDK missing. Install its headers/libraries first; see README.md. "
    "For a nonstandard install set KINOVA_SDK_ROOT to its prefix.")
endif()

add_executable(jaco_cartesian src/cartesian.cpp)
target_compile_features(jaco_cartesian PRIVATE cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(jaco_cartesian PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_include_directories(jaco_cartesian PRIVATE "${KINOVA_API_INCLUDE_DIR}")
target_link_libraries(jaco_cartesian "${KINOVA_COMM_LIBRARY}" "${KINOVA_USB_LIBRARY}")
set_target_properties(jaco_cartesian PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
ament_target_dependencies(jaco_cartesian rclcpp rclcpp_action rcl_interfaces
  sensor_msgs geometry_msgs std_msgs std_srvs tf2 jaco_vla_interfaces)

ament_python_install_package(policy_client PACKAGE_DIR src/policy_client)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES src/dataset_utils.py DESTINATION "${PYTHON_INSTALL_DIR}")
install(TARGETS jaco_cartesian DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/vla_client.py
  DESTINATION lib/${PROJECT_NAME} RENAME vla_client)
install(PROGRAMS scripts/nintendo_server.py
  DESTINATION lib/${PROJECT_NAME} RENAME nintendo_server)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
