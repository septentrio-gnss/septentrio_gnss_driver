cmake_minimum_required(VERSION 3.0.2)
project(septentrio_gnss_driver)

## Compile as C++17, supported in ROS Melodic and newer
add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
message(STATUS "Setting build type to Release as none was specified.")
set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)
# Set the possible values of build type for cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
  "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cpp_common
  rosconsole
  roscpp
  roscpp_serialization
  rostime
  xmlrpcpp
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

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED) # bug with 1.71: spirit is not found ... COMPONENTS system thread regex spirit)
LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

## For PCAP file handling
find_library(libpcap_LIBRARIES pcap)
if ("${libpcap_LIBRARIES}" STREQUAL "pcap-NOTFOUND")
    set(libpcap_FOUND FALSE)
else ()
    set(libpcap_FOUND TRUE)
endif ()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
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

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    diagnostic_msgs
    gps_common
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS cpp_common rosconsole roscpp roscpp_serialization rostime xmlrpcpp message_runtime
   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
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

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node 
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES} 
   ${libpcap_LIBRARIES}
   ${GeographicLib_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files or directories for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
