^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package septentrio_gnss_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.2 (2023-11-19)
------------------
* Commits
   * Merge pull request `#97 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/97>`_ from thomasemter/dev/next
     Integrate README changes from master
   * Fix topics namespace
   * Fix units of imu angular rates
   * Merge upstream README pt2
   * Merge upstream README
   * Update README.md
   * Update README.md
   * Update README.md
   * v1.3.1
   * Updated package.xml
   * v1.3.1
   * Updated package.xml
   * v1.3.1
   * Updated package.xml
   * v1.3.1
   * Updated changelog
   * Merge pull request `#95 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/95>`_ from thomasemter/dev/next
     Fix navsatfix and gpsfix frame ids
   * Update README.md
   * Fix navsatfix and gpsfix frame ids
   * Merge pull request `#92 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/92>`_ from thomasemter/dev/next
     Fix single antenna receiver setup
   * Update changelog
   * Merge
   * Fix single antenna receiver setup
   * Merge pull request `#90 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/90>`_ from thomasemter/dev/next
     Fix empty headers
   * Merge branch 'dev' into dev/next
   * Bump version
   * Fix empty headers
   * Merge pull request `#88 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/88>`_ from thomasemter/dev/next
     Fix navsatfix containing only zeros for INS
   * Align indent
   * Fix navsatfix containig only zeros for INS
   * Merge pull request `#87 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/87>`_ from thomasemter/dev/next
     Update firmware info
   * Reduce INS firmware version info to released version
   * Update firmware info
   * v1.3.0
   * Updated package.xml
   * v1.3.0
   * Update firmware info
   * Updated package.xml
   * v1.3.0
   * Merge pull request `#84 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/84>`_ from thomasemter/dev/next
     Update readme
   * Add expected release dates
   * Add known issues to readme
   * Update version
   * Update readme
   * Merge pull request `#81 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/81>`_ from thomasemter/dev/next
     Fix spelling
   * Improve explanations in readme
   * Categorize stream params
   * Add keep alive check for TCP
   * Fix spelling
   * Change angle wrapping
   * Add TCP communication via static IP server
   * Add units to msgs
   * Fix spelling
   * Merge pull request `#75 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/75>`_ from thomasemter/dev/next
     upcoming release
   * Add heading to GPSFix msg
   * Move constant
   * Change log level of firmware check
   * Add improved VSM handling
   * Change INS in GNSS node detection to auto
   * Fix invald v_x var case
   * Refine readme on UDP
   * Improve server duplicate check
   * Add more info un UDP configuration
   * Fix publish check
   * Add more publishing checks for configured Rx
   * Add const for max udp packet size
   * Update readme and changelog
   * Add device check to node
   * Add checks for IP server duplicates
   * Add latency compensation to att msgs
   * Add device check logic
   * Add UDP params and setup logic
   * Fix multi msg per packet
   * Fix localization stamp and tf publishing
   * Change VSM to be averaged and published with 2 Hz
   * Always publish raw IMU data as indicated
   * Change to empty fields
   * Refine diagnostics naming scheme and add trigger to ensure emission of ReceiverSetup
   * Change diagnostics naming scheme
   * Expand readme on AIM+
   * Reformulate readme about ROS and ROS2
   * Rename msg var
   * Add custom message to report AIM+ status
   * Catch invalid UTM conversion
   * Robustify command reset
   * Add RFStatus diagnostics
   * Merge branch 'dev/next' of https://github.com/thomasemter/septentrio_gnss_driver into dev/next
   * Add VelCovCartesian output
   * Add VelCovCartesian output
   * Refine Rx type check
   * Ensure latency reporting block is activated
   * Add option for latency compensation
   * Fix param type misinterpretation
   * Add missing param to example in readme
   * Add OSNMA diagnostics output
   * Add keep_open option to OSNMA
   * Add OSNMA msg
   * Update changelog
   * Refine README and fix compiled message logic
   * Update changelog
   * Add warning for configuring INS as GNSS
   * Add warn log for misconfiguration
   * Fix pose publishing rate
   * Fix navsatfix publishing
   * Make vars const
   * Replace log functions
   * Add small fixes and cleanup
   * Merge branch 'dev/next' of https://github.com/thomasemter/septentrio_gnss_driver into dev/next
   * Add option to bypass configuration of Rx
   * Add option to bypass confugration of Rx
   * Add diagnostics and time ref msgs again
   * Update README on how to add new messages
   * Add automiatic detection of serial port ID
   * Refine changelog
   * Change invalid timestamp handling for reading from file
   * Add USB serial by ID info to README
   * Fix leap seconds for timestamp if reading from file
   * Fix reconnection logic
   * Replace flow control setup
   * Refactor ioservice
   * notify semaphores in destructor
   * Send port reset only once
   * Fix serial connection
   * Fix talker ID setting for INS
   * Add NMEA talker ID setting to ensure reception
   * Prepare communication for UDP option (still inactive)
   * Fix UDP message identification logic
   * Add test code for UDP, WIP
   * Add UDP client, WIP
   * Set do-not-use for temp to NaN
   * Add processing latency correction for ROS timestamps
   * Fix extsensor parser
   * Add units to remaining msgs
   * Add nodiscard attribute to functions
   * Add nodiscard attribute to functions
   * Add nodiscard attribute to functions
   * Add const specifiers to functions
   * Make settings access const
   * Move SBF ID handling
   * Refactor header assembly
   * Rename message handler again
   * Change parsing utilities and crc to get message
   * Add namespace to enum
   * Change timestamp code
   * Update changelog
   * Change class privacy
   * Add assembled messages, to be tested
   * Add units to come msg definitions
   * Add custom BlockHeader constructor
   * Move wait
   * Remove copy paste vars
   * Add file readers
   * Fix reset main connection on exit hang
   * Fix handling of INS NMEA talker ID
   * Fix error response detection
   * Add packing of generic string messages
   * Exchange concurrent queue
   * Remove obsolete includes
   * Add NMEA handling
   * Change syncronization to semaphore
   * Add message parser, WIP
   * Rearrange io handling, WIP
   * Refactor and cleanup
   * Improve io handling, WIP
   * Refactor message parser, WIP
   * Add message handler
   * Add io modules
   * Add new low level interface, WIP
   * Change connection thread
   * Fix attitude cov flipped twice
   * Add cov alignment from true north to grid north
   * Rename meridian convergence and fix sense
   * Remove obsolete define
   * Fix spelling errors
   * Merge branch 'master' into dev/next
   * v1.2.3
   * Update package.xml
   * v1.2.3
   * Update package.xml
   * v1.2.3
   * Update package.xml
   * v1.2.3
   * Update package.xml
   * Merge pull request `#68 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/68>`_ from thomasemter/master
     dev
   * Fix lat/long in rad
   * Reorder localization msg filling
   * Update readme
   * Fix NED to ECEF rotation matrix
   * Add localization ECEF publishing
   * Merge branch 'dev/next' of https://github.com/thomasemter/septentrio_gnss_driver into dev/next
   * Merge branch 'dev/next' of https://github.com/thomasemter/septentrio_gnss_driver into dev/next
   * Merge branch 'dev/next' of https://github.com/thomasemter/septentrio_gnss_driver into dev/next
   * Add ecef localization msg
   * Add local to ecef transforms
   * Change default datum to Default
   * Clean up block data size defines
   * Change default datum to WGS84
   * Set correct value for max number of base vector info
   * Add check on shutdown to only close ports on hardware
   * Refine readme
   * Merge branch 'master' of https://github.com/thomasemter/septentrio_gnss_driver
   * Add missing param
   * Add possibility to use multiple RTK corrections of the same type
   * Only set baud rates on real serial ports
   * Fix decimal places trimming
   * Update changelog
   * Merge branch 'dev/next'
   * Add base vecotr info to README
   * Change param to empty vector
   * Change param to empty vector
   * Fix template argument
   * Add quotation marks to pw if it contains spaces
   * Add quotation marks to pw if it contains spaces
   * Add option to keep aiding connections open on shutdown
   * Merge branch 'master' into dev/next
   * Add option to keep aiding connections open on shutdown
   * Disable ntrip on shutdown
   * Disable ntrip on shutdown
   * Add base vector callbacks and publishing, WIP
   * Add base vector msgs and parsers, WIP
   * Fix comment swap
   * Add send_gga option to serial RTK and fix IP server id
   * Add possibility to specify IP server connection
   * Increase version number in package.xml and harmonize it with ROS2
   * Reset main port to auto input
   * Add reset all used ports on shutdown
   * Improve readme
   * Change vsm options to allow simultaneous input
   * Change corrections settings to receiver simultaneously
   * Change correction options to be used simultenously
   * Change param name for future extensibility
   * Change param name for future extensibility
   * Rework corretion parameters and add more flexible options
   * Fix some spelling in readme
   * Add receiver type INS as GNSS
   * Add option to use external VSM input
   * Add more log output for vsm
   * Add VSM from odometry or twist ROS messages
   * Fix GPGGA and GPRMC timestamp from GNSS
   * Use only one stream for NMEA messages
   * Fix merge error
   * Fix merge error
   * Add all possible periods and rework validity check
   * Update changelog
   * Add 5 ms period option
   * Fix changelog
   * Add twist output
   * Add missing files to clang-formatting
   * Merge branch 'dev/next'
   * Merge branch 'master' of https://github.com/thomasemter/septentrio_gnss_driver
   * Format according to clang-format
   * Change log level of local frame tf insertion
   * Always register ReceiverSetup
   * Add firmware checks
   * Add log and info to README
   * Add insertion of local frame
   * Update README and CHANGELOG
   * Use leap seconds from receiver
   * Update changelog
   * Add config files for GNSS and INS
   * Add ReceiverTime, WIP
   * Add configs for GNSS and INS
   * Contributors: Thomas Emter, Tibor Dome, septentrio-users, tibordome

1.3.1 (2023-07-06)
------------------
* New Features
   * Recovery from connection interruption
   * Add option to bypass configuration of Rx
   * OSNMA
   * Latency compensation for ROS timestamps
   * Output of SBF block VelCovCartesian
   * Support for UDP and TCP via IP server
   * New VSM handling allows for unknown variances (INS firmware >= 1.4.1)
   * Add heading angle to GPSFix msg (by diverting dip field, cf. readme)
* Improvements
   * Rework IO core and message handling
     * Unified stream processing 
     * Internal data queue
     * Prevent message loss in file reading
   * Add some explanatory warnings for parameter mismatches
   * Add units to message definitions
* Fixes
   * navsatfix for INS
   * Empty headers
   * Single antenna receiver setup
* Preliminary Features
   * Output of localization and tf in ECEF frame, testing and feedback welcome

1.2.3 (2022-11-09)
------------------
* New Features
   * Twist output option
   * Example config files for GNSS and INS
   * Get leap seconds from receiver
   * Firmware check
   * VSM from odometry or twist ROS messages
   * Add receiver type in case INS is used in GNSS mode
   * Add publishing of base vector topics
* Improvements
   * Rework RTK corrections parameters and improve flexibility
* Fixes
   * /tf not being published without /localization
   * Twist covariance matrix of localization
   * Support 5 ms period for IMU explicitly

1.1.2 (2022-06-22)
------------------
* Fixes
   * Memory corruption under adverse conditions

1.1.1 (2022-05-16)
------------------
* New Features
   * Add login credentials
   * Activate NTP server if use_gnss_time is set to true
* Improvements
   * Add NED option to localization
* Fixes
   * IMU orientation for ROS axis convention
* Commits
    * Merge pull request `#62 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/62>`_ from thomasemter/dev/next
      Small fixes and additions
    * Amend readme regarding robot_localization
    * Add more explanations for IMU orientation in ROS convention
    * Fix formatting in readme
    * Fix package name in readme
    * Update readme
    * Update changelog
    * Fix IMU orientation for ROS axis orientation
    * Activate NTP only if GNSS time is used
    * Add NED option to localization
    * Set NMEA header to
    * Fix logging causing crash
    * Update readme and changelog
    * Activate NTP server
    * Add credentials for access control
    * Contributors: Thomas Emter, Tibor Dome

1.1.0 (2022-04-25)
------------------
* New Features
   * Add option to use ROS axis orientations according to REP103
   * Add frame_id parameters
   * Add option to get frames from tf
   * Publishing of cartesian localization in UTM (topic and/or tf) for INS
   * Publishing of IMU topic for INS
   * Publishing of MeasEpoch
   * ROS2 branch
* Improvements
   * Add multi antenna option
   * Increase number of SBF streams
   * Add option to set polling_period to "on change"
   * Increased buffer size from 8192 to 131072 bytes
   * Add endianess aware parsers
   * Only publish topics set to true
   * Add parameter to switch DEBUG logging on and off
   * Change GPxxx messages to ROS built-in types
   * Remove duplicate INS msg types
* Fixes
   * Setting of antenna type
   * Publishing rate interconnections of gpsfix and velcovgeodetic
   * Missing quotes for antenna type
   * Broken attitude parsing pose and gpsfix from INS
   * IMU orientation was not sent to Rx
   * Graceful shutdown of threads
* Commits
    * Merge branch 'dev'
    * Prepare new release
    * Prepare new release
    * Merge pull request `#53 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/53>`_ from thomasemter/dev/refactor
      Very last changes
    * Add geographic lib dependency to package.xml
    * Add comment for frame of main antenna
    * Move utm zone locking section in readme
    * Reformulate readme section on frames
    * Merge pull request `#52 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/52>`_ from thomasemter/dev/refactor
      Last changes
    * Change frame id back to poi_frame_id
    * Make error log more explicit
    * Merge pull request `#49 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/49>`_ from thomasemter/dev/refactor
      Improve IMU blocks sync and do-not-use value handling
    * Fix buffer size in changelog
    * Turn off Nagle's algorithm for TCP
    * Fix changelog formatting
    * Fix readme
    * Set default base frame to base_link
    * Fix valid tow check logic
    * Increase buffer size for extreme stress tests
    * Fix crc check
    * Fix and streamline tf handling
    * Add checks for validity of values
    * Fix rad vs deg
    * Update changelog
    * Add some comments
    * Set stdDevMask to values > 0.0 in node
    * Set stdDevMask to values > 0.0
    * Add info on RNDIS and set it to default
    * Increase default serial baud rate
    * Add parameter to set log level to debug
    * Change defaults for publishers in node
    * Put publish params together and fix mismatch in readme
    * Improve IMU blocks sync and do-not-use value handling
    * Merge pull request `#48 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/48>`_ from thomasemter/dev/refactor
      Fix measepoch not publishing without gpsfix
    * Fix measepoch not publishing without gpsfix
    * Merge pull request `#47 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/47>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Publish only messages set to true
    * Remove leftover declaration
    * Merge branch 'dev/endianess_agnostic' into dev/refactor
    * Update readme to reflect endianess aware parsing
    * Remove msg smart pointers
    * Fix array assertion failure
    * Cleanup
    * Add ReceiverStatus parser
    * Add QualityInd parser
    * Add DOP parser
    * Add ReceiverSetup parser
    * Fix MeasEpoch and ChannelStatus parsers, add measepoch publishing
    * Add ChannelStatus parser
    * Add MeasEpoch parser
    * Add IMU and VelSensor setup parsers
    * Add Cov SBF parsers
    * Add templated qi parser function
    * Add AttEuler+Cov parser
    * Revert ordering change inside INSNav ROS msgs
    * Add ExtSensorMeas parser
    * Add PVT parsers
    * Add range checks to parsers
    * Replace INSNav grammar with parsers
    * Test parser vs. grammar for better performance
    * Fix sb_list check
    * Add IMU and VelSensor setup grammars
    * Move adapt ROS header to typedefs.h
    * Add revision check to MeasEpoch
    * Fix ReceiverStatus grammar
    * Extend ReceiverSetup and add revision checks
    * Change logger and fix loop range
    * Remove reserved bytes from parsing
    * Remove obsolete structs
    * Directly parse Cov SBFs to ROS msg
    * Directly parse PVT SBFs, remove obsolete ids
    * Rename rev to revision
    * Fix block header parsing
    * Directly parse AttEuler to ROS msg
    * Directly parse to ROS msgs for INSNavXxx
    * Exchange pow with square function and remove casts
    * Merge pull request `#46 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/46>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Simplify sync bytes check
    * Move tow/wnc to BlockHeader
    * Adjust order in INSNav ros msgs
    * Fix INSNav grammars
    * Change BlockHeader structure
    * Remove length ref from header
    * Rectify sb_list check of INSNavXxx
    * Add automtatic activation of multi-antenna mode
    * Merge branch 'dev/refactor' of https://github.com/thomasemter/septentrio_gnss_driver into dev/refactor
    * Add automtatic activation of multi-antenna mode
    * Fix wrong scope of phoenix::ref variables
    * Fix AttEuler grammar
    * Add max size checks to QualityInd and ReceiverStatus
    * Replace locals with phoenix::ref in grammars
    * Add revision dependent parsing to PVTs
    * Change offset check to epsilon
    * Change offset check to epsilon
    * Fix parsing checks
    * Set has arrived to false on parsing error
    * Add INSNav grammars
    * Add abs to offset check
    * Add abs to offset check
    * Add Cov grammars
    * Remove superfluous typdefs of structs
    * Add ReceiverStatus grammar
    * Add QualityINd grammar
    * Merge pull request `#45 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/45>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Add id check to header grammar
    * Add id check to header grammar
    * Add ReceiverSetup grammar
    * Add DOP grammar
    * Directly intialize vector to parse
    * Add MeasEpoch grammar
    * Remove duplicate msg types
    * Remove obsolete include
    * Add revision and length return to header grammar
    * Merge branch 'feature/endianess_agnostic' into dev/endianess_agnostic
    * Make multi_antenna option also usable for gnss
    * Add typedefs plus some minor changes
    * Add warning concerning pitch angle if antennas are rotated
    * Add multi antenna option to ins and fix antenna offset decimal places trimming
    * Fix identation
    * Distinguish between gnss and ins for spatial config from tf
    * Merge pull request `#43 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/43>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Add vehicle frame for clarity
    * Handle missing tf more gently
    * Merge branch 'dev/spatial_config_via_tf' into dev/refactor
    * Update readme
    * Fix antenna offset from tf
    * Add automatic publishing of localization if tf is activated
    * Add automatic publishing of localization if tf is activated
    * Add spatial config via tf, to be tested
    * Fix crashes due to parsing errors (replacing uncatched throws)
    * Add tf broadcasting
    * Add comments
    * Add localization in UTM output
    * Add check to IMU msg sync
    * Change msg sync to allow for 200 Hz IMU msgs
    * Add ROS IMU msg
    * Fix IMU setup message attitude conversion
    * Fix pose from INS data
    * Fix IMU raw data rotation compensation
    * Make antenna attitude offset usable by GNSS
    * Add ros directions option to pose and fix covariances
    * Update readme
    * Merge branch 'feature/ros_axis_orientation' into dev/refactor
    * Add nmea_msgs dependencies
    * Merge branch 'dev/nmea' into dev/refactor
    * Update readme
    * Update readme
    * Add antenna offsets to conversions
    * Fix IMU orientation conversion
    * Change ExtSensorMead temperature to deg C
    * Add axis orientation info to readme
    * Fix IMU axis orientation
    * Change get int param
    * Update readme to reflect removal of aux antenna offset
    * Fix different antenna setup message for INS and remove obsolete aux1 antenna offset for GNSS
    * Fix ExtSensorMeas message filling
    * Fix ExtSensorMeas message to reflect available fields
    * Fix missing INS blocks
    * Fix missing INS blocks
    * WIP, introduce ros axis orientation option, to be tested
    * Add option to set pvt rate to OnChange
    * Add comment on NTP to readme
    * Change to nmea_msgs
    * Add automatic addition of needed sub messages
    * Comment out setting debug level
    * Add comments and fix spelling errors
    * Merge pull request `#42 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/42>`_ from thomasemter/dev/refactor
      Dev/refactor
    * Change to quaternion msg typedef
    * Comment out debug logging
    * Remove filling of seq field
    * Change msg definitions to be compatible with ROS2
    * Update readme
    * Change make_shared for portability and add more typedefs
    * Add get param int fallback for numeric antenna serial numbers
    * Change Attitude to be published with pvt rate
    * Add log identifier
    * Add checks for relevant ros params
    * Concatenate multiple SBF blocks in streams
    * Move main into own file
    * Move get ros time to AsyncManager
    * Remove obsolete param comment
    * Move get ros params to base class
    * Change to nsec timestamp internally
    * Add publishing functionality to node base class
    * Move node handle ptr and functions to base class and rename
    * Add stamp to nmea parsing
    * Add logging in PcapReader
    * Add logging in CircularBuffer
    * Add missed logging
    * Add logging in AsyncManager
    * Add getTime function
    * Add logging in RxMessage
    * Add logging in CallbackHandlers
    * Add log function to node by polymorphism, logging in Comm_OI
    * Fix wait function and force use_gnss_time when reading from file
    * Add thread shutdown and remove spurious delete
    * Add typedefs for ins messages
    * Add typedefs for gnss messages
    * Add typedefs for ros messages
    * Refine shutdown
    * Fix shutdown escalating to SIGTERM
    * Move waiting for response in send function
    * Make functions private
    * Change crc to C++
    * Fix variable name
    * Remove global variables from node cpp file
    * Move more global settings to settings struct
    * Move more global settings to settings struct
    * Move global settings to settings struct
    * Move more functions to CommIo
    * Move settings to struct and configuration to CommIo
    * Merge branch 'dev/change_utc_calculation' into dev/refactor
    * Remove obsolete global variables
    * Move g_unix_time to class
    * Make has_arrived booleans class memebers and rx_message a persistent class
    * Make node handle a class member
    * Fix parsing of ID and rev
    * Finish ChannelStatusGrammar, to be tested
    * WIP, partially fix ChannelStatusGrammar
    * Add SBF length parsing utility
    * Insert spirit parsers
    * WIP, add omission of padding bytes
    * WIP, add more spirit parsers
    * Add parsing utilities for tow, wnc and ID
    * Move getId/Tow/Wnc to parsing utilities
    * Change UTC calculation to use tow and wnc
    * WIP, add boost spirit and endian buffers
    * Change UTC calculation to use tow and wnc
* Update Readme and Changelog
* Contributors: Thomas Emter, Tibor Dome

1.0.8 (2021-10-23)
------------------
* Added INS Support

1.0.7 (2021-05-18)
------------------
* Clang formatting, publishing from SBF log, play-back of PCAP files

1.0.6 (2020-10-16)
------------------
* ROSaic binary installation now available on Melodic & Noetic

1.0.5 (2020-10-15)
------------------
* changed repo name
* v1.0.4
* 1.0.3
* Merge pull request `#22 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/22>`_ from septentrio-gnss/local_tibor
  New changelog
* New changelog
* Merge pull request `#21 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/21>`_ from septentrio-gnss/local_tibor
  Added rosdoc.yaml file
* Merge pull request `#20 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/20>`_ from septentrio-gnss/local_tibor
  Improved doxygen annotations
* Merge pull request `#19 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/19>`_ from septentrio-gnss/local_tibor
  Improved doxygen annotations
* Update README.md
* Merge pull request `#18 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/18>`_ from septentrio-gnss/local_tibor
  Adopted ROS and C++ conventions, added ROS diagnostics msg,
* Update README.md
* Update README.md
* Update README.md
* Contributors: septentrio-users, tibordome

1.0.4 (2020-10-11)
------------------
* Added rosdoc.yaml file
* Improved doxygen annotations
* Improved doxygen annotations
* Adopted ROS and C++ conventions, added ROS diagnostics msg, removed ROS garbage value bug, added auto-detection of SBF arrival order for composite ROS msgs
* Merge branch 'master' of https://github.com/septentrio-gnss/rosaic
* NTRIP with Datalink, circular buffer, reading connection descriptor, new messages
* Update README.md
* Contributors: septentrio-users, tibordome

1.0.3 (2020-09-30)
------------------
* Add new config/rover.yaml file
* Add config/rover.yaml to .gitignore
* Merge pull request `#17 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/17>`_ from septentrio-gnss/local_tibor
  NTRIP with Datalink, circular buffer, reading connection descriptor..
* Merge branch 'local_tibor'
* NTRIP with Datalink, circular buffer, reading connection descriptor, new messages
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#16 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/16>`_ from septentrio-gnss/local_tibor
  NTRIP parameters added, reconnect_delay_s implemented,
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#15 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/15>`_ from tibordome/local_tibor
  GPSFix completed, datum as new parameter
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#14 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/14>`_ from tibordome/local_tibor
  GPSFix completed, datum as new parameter
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#13 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/13>`_ from tibordome/local_tibor
  Added AttCovEuler.msg and AttEuler.msg
* Merge pull request `#12 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/12>`_ from tibordome/local_tibor
  Fixed service field of NavSatStatus
* Contributors: Tibor Dome, septentrio-users, tibordome

1.0.2 (2020-09-25)
------------------
* NTRIP parameters added, reconnect_delay_s implemented, package.xml updated, ROSaic now detects connection descriptor automatically, mosaic serial port parameter added
* GPSFix completed, datum as new parameter, ANT type and marker-to-arp distances as new parameters, BlockLength() method corrected, sending multiple commands to Rx corrected by means of mutex
* Contributors: tibordome

1.0.1 (2020-09-22)
------------------
* GPSFix completed, datum as new parameter, ANT type and marker-to-arp distances as new parameters, BlockLength() method corrected, sending multiple commands to Rx corrected by means of mutex
* Added AttCovEuler.msg and AttEuler.msg
* Fixed service field of NavSatStatus, fixed ROS header's seq field of each published ROS message, added write method for sending commands to Rx, successfully tested, added AttEuler, added AttCovEuler
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#11 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/11>`_ from tibordome/local_tibor
  rosconsole_backend_interface dependency not needed
* rosconsole_backend_interface dependency not needed
* Merge pull request `#10 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/10>`_ from tibordome/local_tibor
  rosconsole_log4cxx dep not needed
* rosconsole_log4cxx dep not needed
* Merge pull request `#9 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/9>`_ from tibordome/local_tibor
  rosconsole_log4cxx dep not needed
* rosconsole_log4cxx dep not needed
* Merge pull request `#8 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/8>`_ from tibordome/local_tibor
  Local tibor
* Update README.md
* Merge pull request `#7 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/7>`_ from tibordome/local_tibor
  Ready for First Release
* Update README.md
* Update README.md
* Update README.md
* Merge pull request `#6 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/6>`_ from tibordome/local_tibor
  Local tibor
* Merge pull request `#5 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/5>`_ from tibordome/local_tibor
  TCP seems to work
* Contributors: Tibor Dome, tibordome

1.0.0 (2020-09-11)
------------------
* Ready for first release
* Added Gpgga.msg and PosCovGeodetic.msg files
* Ready for First Release
* Ready for first release
* Ready for first release
* Ready for first release
* TCP bug removed
* TCP bug removed
* TCP seems to work
* Merge pull request `#4 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/4>`_ from tibordome/v0.2
  V0.2
* PVTCartesian and PVTGeodetic publishing works on serial
* PVTCartesian and PVTGeodetic publishing works on serial
* Merge pull request `#3 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/3>`_ from tibordome/v0.2
  Add doxygen_out and Doxyfile 2nd trial
* Add doxygen_out and Doxyfile 2nd trial
* Merge pull request `#2 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/2>`_ from tibordome/v0.1
  Add doxygen_out and Doxyfile
* Add doxygen_out and Doxyfile
* Update README.md
* Create README.md
* Update LICENSE
* Merge pull request `#1 <https://github.com/septentrio-gnss/septentrio_gnss_driver/issues/1>`_ from tibordome/add-license-1
  Create LICENSE
* Create LICENSE
* Create LICENSE
* Commit
* Successfully tested publishing to /gpgga topic via serial
* To make sure master branch exists
* Contributors: Tibor Dome, tibordome
