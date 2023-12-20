cmake_minimum_required(VERSION 3.5)
project(septentrio_gnss_driver)

## Compile as C++17
add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
message(STATUS "Setting build type to Release as none was specified.")
set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)
# Set the possible values of build type for cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
  "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(WARNFLAGS "-Wall -Wextra -Wpedantic -Wcast-align -Wnon-virtual-dtor -Woverloaded-virtual -Wcast-qual -Wctor-dtor-privacy")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

## For PCAP file handling
find_library(libpcap_LIBRARIES pcap)
if ("${libpcap_LIBRARIES}" STREQUAL "pcap-NOTFOUND")
    set(libpcap_FOUND FALSE)
else ()
    set(libpcap_FOUND TRUE)
endif ()

if("$ENV{ROS_VERSION}" STREQUAL "1")
  ## ROS 1 -----------------------------------------------------------------------------------------------  
  add_compile_definitions(ROS1)

  configure_file(${CMAKE_SOURCE_DIR}/msg_cache/BlockHeaderRos1.msg ${CMAKE_SOURCE_DIR}/msg/BlockHeader.msg COPYONLY)

  ## Dependencies
  find_package(catkin REQUIRED COMPONENTS
  roscpp
  nmea_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  gps_common
  message_generation
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  )

  ## Declare messages
  add_message_files(
    FILES
    AIMPlusStatus.msg
    BaseVectorCart.msg
    BaseVectorGeod.msg
    BlockHeader.msg
    GALAuthStatus.msg
    RFBand.msg
    RFStatus.msg
    MeasEpoch.msg
    MeasEpochChannelType1.msg
    MeasEpochChannelType2.msg
    PVTCartesian.msg
    PVTGeodetic.msg
    PosCovCartesian.msg
    PosCovGeodetic.msg
    ReceiverTime.msg
    VelCovCartesian.msg
    VelCovGeodetic.msg
    AttEuler.msg
    AttCovEuler.msg
    INSNavCart.msg
    INSNavGeod.msg
    IMUSetup.msg
    VectorInfoCart.msg
    VectorInfoGeod.msg
    VelSensorSetup.msg
    ExtSensorMeas.msg
  )
  ## Generate messages
  generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    diagnostic_msgs
    gps_common
  )

  # Catkin package
  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS roscpp message_runtime
   DEPENDS Boost
  )

  # Build
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
  )
  add_executable(${PROJECT_NAME}_node
    src/septentrio_gnss_driver/communication/communication_core.cpp
    src/septentrio_gnss_driver/communication/message_handler.cpp 
    src/septentrio_gnss_driver/communication/telegram_handler.cpp
    src/septentrio_gnss_driver/crc/crc.cpp
    src/septentrio_gnss_driver/node/rosaic_node_ros1.cpp
    src/septentrio_gnss_driver/parsers/nmea_parsers/gpgga.cpp 
    src/septentrio_gnss_driver/parsers/nmea_parsers/gprmc.cpp 
    src/septentrio_gnss_driver/parsers/nmea_parsers/gpgsa.cpp 
    src/septentrio_gnss_driver/parsers/nmea_parsers/gpgsv.cpp
    src/septentrio_gnss_driver/parsers/parsing_utilities.cpp 
    src/septentrio_gnss_driver/parsers/string_utilities.cpp  
  )
  add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_node 
    ${catkin_LIBRARIES}
    boost_regex # ordering is important for regex https://stackoverflow.com/questions/17588440/cannot-link-boost-regex
    ${Boost_LIBRARIES} 
    ${libpcap_LIBRARIES}
    ${GeographicLib_LIBRARIES}
  )
 
  # Installation
  install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  #-------------------------------------------------------------------------------------------------------
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  ## ROS 2 -----------------------------------------------------------------------------------------------
  add_compile_definitions(ROS2)

  configure_file(${CMAKE_SOURCE_DIR}/msg_cache/BlockHeaderRos2.msg ${CMAKE_SOURCE_DIR}/msg/BlockHeader.msg COPYONLY)

  # Dependencies
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
  
  ## Declare  messages
  rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AIMPlusStatus.msg"
  "msg/BaseVectorCart.msg"
  "msg/BaseVectorGeod.msg"
  "msg/BlockHeader.msg"
  "msg/GALAuthStatus.msg"
  "msg/RFBand.msg"
  "msg/RFStatus.msg"
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
  src/septentrio_gnss_driver/communication/communication_core.cpp
  src/septentrio_gnss_driver/communication/message_handler.cpp 
  src/septentrio_gnss_driver/communication/telegram_handler.cpp
  src/septentrio_gnss_driver/crc/crc.cpp
  src/septentrio_gnss_driver/node/main.cpp
  src/septentrio_gnss_driver/node/rosaic_node.cpp
  src/septentrio_gnss_driver/parsers/nmea_parsers/gpgga.cpp 
  src/septentrio_gnss_driver/parsers/nmea_parsers/gprmc.cpp 
  src/septentrio_gnss_driver/parsers/nmea_parsers/gpgsa.cpp 
  src/septentrio_gnss_driver/parsers/nmea_parsers/gpgsv.cpp
  src/septentrio_gnss_driver/parsers/parsing_utilities.cpp 
  src/septentrio_gnss_driver/parsers/string_utilities.cpp 
  )
  target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"  
  "$<INSTALL_INTERFACE:include>"
  )
  target_link_libraries(${library_name}
  boost_regex # ordering is important for regex https://stackoverflow.com/questions/17588440/cannot-link-boost-regex
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

  # Testing
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  # Export Targets
  ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
  ament_export_dependencies(${dependencies} rosidl_default_runtime)

  # Installation
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
  ##-------------------------------------------------------------------------
else()
  message(FATAL_ERROR "ROS 1 or ROS 2 not found! Please, source your ROS installation.")
endif()

