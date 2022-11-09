cmake_minimum_required(VERSION 3.5)
project(septentrio_gnss_driver)

## Compile as C++17
add_compile_options(-std=c++17)

## ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread regex chrono)
LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

## For PCAP file handling
find_library(libpcap_LIBRARIES pcap)
if ("${libpcap_LIBRARIES}" STREQUAL "pcap-NOTFOUND")
    set(libpcap_FOUND FALSE)
else ()
    set(libpcap_FOUND TRUE)
endif ()


## Declare  messages
rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/BaseVectorCart.msg"
   "msg/BaseVectorGeod.msg"
   "msg/BlockHeader.msg"
   "msg/MeasEpoch.msg"
   "msg/MeasEpochChannelType1.msg"
   "msg/MeasEpochChannelType2.msg"
   "msg/PVTCartesian.msg"
   "msg/PVTGeodetic.msg"
   "msg/PosCovCartesian.msg"
   "msg/PosCovGeodetic.msg"
   "msg/ReceiverTime.msg"
   "msg/VelCovCartesian.msg"
   "msg/VelCovGeodetic.msg"
   "msg/AttEuler.msg"
   "msg/AttCovEuler.msg"
   "msg/INSNavCart.msg"
   "msg/INSNavGeod.msg"
   "msg/IMUSetup.msg"
   "msg/VectorInfoCart.msg"
   "msg/VectorInfoGeod.msg"
   "msg/VelSensorSetup.msg"
   "msg/ExtSensorMeas.msg"
   DEPENDENCIES std_msgs
)

set(library_name septentrio_gnss_driver_core)
set(executable_name septentrio_gnss_driver_node)

set(dependencies
   rclcpp
   rclcpp_components
   diagnostic_msgs
   nmea_msgs
   sensor_msgs
   geometry_msgs
   gps_msgs
   nav_msgs
   tf2
   tf2_eigen
   tf2_geometry_msgs
   tf2_ros
)

## shared library
add_library(${library_name} SHARED
   src/septentrio_gnss_driver/node/rosaic_node.cpp
   src/septentrio_gnss_driver/communication/circular_buffer.cpp 
   src/septentrio_gnss_driver/parsers/parsing_utilities.cpp 
   src/septentrio_gnss_driver/parsers/string_utilities.cpp 
   src/septentrio_gnss_driver/parsers/nmea_parsers/gpgga.cpp 
   src/septentrio_gnss_driver/parsers/nmea_parsers/gprmc.cpp 
   src/septentrio_gnss_driver/parsers/nmea_parsers/gpgsa.cpp 
   src/septentrio_gnss_driver/parsers/nmea_parsers/gpgsv.cpp 
   src/septentrio_gnss_driver/crc/crc.cpp
   src/septentrio_gnss_driver/communication/communication_core.cpp 
   src/septentrio_gnss_driver/communication/rx_message.cpp 
   src/septentrio_gnss_driver/communication/callback_handlers.cpp
   src/septentrio_gnss_driver/communication/pcap_reader.cpp
)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"  
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${library_name}
  ${Boost_LIBRARIES}
  pcap
  ${GeographicLib_LIBRARIES}
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(${library_name}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${library_name} "${cpp_typesupport_target}")
  add_compile_definitions(ROS2_VER_N250)
endif()

## executable
add_executable(${executable_name}
  src/septentrio_gnss_driver/node/main.cpp
)
target_link_libraries(${executable_name}
  ${library_name}
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "rosaic_node::ROSaicNode")

# Enable testing with colcon build --cmake-args -DBUILD_TESTING=ON
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#
#  add_subdirectory(test)
#endif()

# Export Targets
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} rosidl_default_runtime)

# installation
install(TARGETS
  ${library_name} EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} EXPORT ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
